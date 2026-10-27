// SPDX-License-Identifier: Apache-2.0
// XR-DAP-F103: a CMSIS-DAP v2 debug probe on STLINK-V2 style STM32F103 boards, built on
// LibXR. One USB full-speed Bulk v2 DAP interface with the XR vendor extension
// (multi-address access, pointer chains, probe-side sampling); the red/blue light on PA9
// shows the host state and DAP activity (see status_led.hpp).
//
// XR-DAP-F103：基于 LibXR、跑在 STLINK-V2 形态 STM32F103 板子上的 CMSIS-DAP v2 调试探针。
// 一个 USB 全速 Bulk v2 DAP 接口，带 XR 厂商扩展（多地址读写、指针链、探针端采样）；PA9
// 上的红蓝灯显示上位机状态和 DAP 活动（见 status_led.hpp）。

#include "app_main.h"

#include <cstdint>

#include "daplink_v2_profile_swd.hpp"
#include "debug/swd_general_gpio.hpp"
#include "libxr.hpp"
#include "main.h"
#include "probe_info.hpp"
#include "status_led.hpp"
#include "stm32_gpio.hpp"
#include "stm32_timebase.hpp"
#include "stm32_usb_fsdev.hpp"
#include "tick_timer.hpp"
#include "xrdap/activity/dap_activity.hpp"
#include "xrdap/light/status_light.hpp"
#include "chip_uid.hpp"
#include "xrdap/probe/family.hpp"
#include "xrdap/session/executor.hpp"
#include "xrdap/session/session_owner.hpp"
#include "xrdap/swd/f1_gpio.hpp"
#include "xrdap/vendor/dap_vendor.hpp"

namespace
{
using namespace XrDap;

constexpr uint16_t kDapPacketSize = 512;
constexpr uint8_t kDapPacketCount = 4;
// SWD clock timing of this board in tenths of a CPU cycle (XR_CLOCK_MODE is the system
// clock in MHz, set by CMake): one step of the delay count adds 9.0 cycles to an SWCLK
// period, a delayed period takes 16.9 cycles plus the steps, and a period without delay
// 11.4 cycles. Typical values measured on hardware with the cycle counter on every
// transfer path at both clock profiles; libxr picks the step nearest to the request.
constexpr uint32_t kCpuHz = XR_CLOCK_MODE * 1000000U;
constexpr LibXR::Debug::SwdGpioTiming kSwdTiming{90U, 169U, 114U};

constexpr uint8_t PinIndex(uint16_t mask) { return __builtin_ctz(mask); }
// SWCLK and SWDIO are PB13 and PB14 (CubeMX labels SWCLK / SWDIO).
using Swclk = F1Gpio<GPIOB_BASE, PinIndex(SWCLK_Pin)>;
using Swdio = F1Gpio<GPIOB_BASE, PinIndex(SWDIO_Pin)>;
using Swd = LibXR::Debug::SwdGeneralGPIO<Swclk, Swdio>;
using DapBase =
    LibXR::USB::DapLinkV2Class<Swd, kDapPacketSize, kDapPacketCount, kDapPacketSize>;

// The DAP class, also counting USB bus resets (libxr reports them to every class):
// a changed count ends the USB session (and its sampling), as the host is gone.
class UsbDap final : public DapBase
{
 public:
  using DapBase::DapBase;

  static uint32_t BusResets(void* ctx) { return static_cast<UsbDap*>(ctx)->resets_; }

 private:
  void OnDeviceEvent(bool in_isr, LibXR::USB::DeviceEvent event,
                     uint8_t endpoint) override
  {
    if (event == LibXR::USB::DeviceEvent::RESET)
    {
      resets_ = resets_ + 1U;
    }
    DapBase::OnDeviceEvent(in_isr, event, endpoint);
  }

  volatile uint32_t resets_ = 0;  // written by the USB interrupt, read by the main loop
};

// FSDEV packet memory is 512 B: 8 B of BTABLE per endpoint, then 64 B per direction.
// The 64 passed with each buffer below is that packet memory share; the arrays are the
// LibXR slot memory in SRAM, two slots of one whole DAP packet each for EP1.
// LibXR endpoint slot memory must be aligned to HW_CACHE_LINE_SIZE.
constexpr size_t kFsPacket = 64;
alignas(LibXR::HW_CACHE_LINE_SIZE) uint8_t ep0_buffer[kFsPacket];
alignas(LibXR::HW_CACHE_LINE_SIZE) uint8_t ep1_dap_in[2 * kDapPacketSize];
alignas(LibXR::HW_CACHE_LINE_SIZE) uint8_t ep1_dap_out[2 * kDapPacketSize];
}  // namespace

extern "C" void app_main(void)
{
  static LibXR::STM32Timebase timebase;
  LibXR::PlatformInit();

  static Swclk swclk;
  static Swdio swdio;
  // nRESET (PB6) is open drain and released by MX_GPIO_Init: the board pulls the line
  // high, so the probe never resets the target unless the host asks for it.
  static LibXR::STM32GPIO nreset(NRST_GPIO_Port, NRST_Pin);
  static Swd swd(swclk, swdio, kCpuHz, kSwdTiming);

  static UsbDap dap(LibXR::USB::Endpoint::EPNumber::EP1,
                    LibXR::USB::Endpoint::EPNumber::EP1, swd, &nreset);
  static char serial[SerialBufferSize(kChipUidBytes)];
  FormatSerial(ChipUid(), kChipUidBytes, serial);
  SetProbeInfo(dap, {kProductName, kBoardName, serial, kFirmwareVersion});
  // USB is the only transport, so the USB host always owns the session; it ends on
  // DAP_Disconnect and on a USB bus reset.
  static SessionOwner session;
  session.SetUsbResetSource(&UsbDap::BusResets, &dap);
  static DapVendor vendor(swd, session);
  static DapActivity activity;
  static Executor<UsbDap> exec(dap, session, vendor, activity);
  static Tim2TickTimer tick(vendor);

  static constexpr auto kStrings = LibXR::USB::DescriptorStrings::MakeLanguagePack(
      LibXR::USB::DescriptorStrings::Language::EN_US, kVendorName, kUsbProductName,
      kSerialPrefix);
  static LibXR::STM32USBDeviceDevFs usb(
      {{ep0_buffer, kFsPacket}, {ep1_dap_out, kFsPacket}},
      {{ep0_buffer, kFsPacket}, {ep1_dap_in, kFsPacket}},
      LibXR::USB::DeviceDescriptor::PacketSize0::SIZE_64, Family::kUsbVid,
      Family::kUsbPid, Family::kBcdF103, {&kStrings}, {{&dap}},
      {ChipUid(), kChipUidBytes});
  usb.Init(false);
  usb.Start(false);

  static StatusLed led;
  static Light::StatusLight light(
      static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds()));
  // LibXR runs DAP commands (and the executor's hook) when the executor calls Step().
  // Between commands the vendor extension samples a tick when TIM2 counted one, so Step
  // does not wait while sampling runs.
  for (;;)
  {
    vendor.Poll(tick);
    (void)exec.StepUsb(vendor.Sampling() ? 0 : 1);
    const auto now_ms = static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    const auto link = dap.IsInited() ? Light::Link::WIRED : Light::Link::NONE;
    led.Show(light.Update(link, Light::Setting::NONE, activity.Read(now_ms), now_ms));
  }
}
