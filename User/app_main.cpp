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

#include "dap_vendor.hpp"
#include "daplink_v2_profile_swd.hpp"
#include "debug/swd_general_gpio.hpp"
#include "fast_gpio.hpp"
#include "libxr.hpp"
#include "main.h"
#include "probe_info.hpp"
#include "status_led.hpp"
#include "status_light.hpp"
#include "stm32_gpio.hpp"
#include "stm32_timebase.hpp"
#include "stm32_usb_fsdev.hpp"

namespace
{
using namespace XrDap;

constexpr uint16_t kUsbVid = 0x1D50;  // XRUSB VID:PID, shared by the XR-DAP family
constexpr uint16_t kUsbPid = 0x6199;
constexpr uint16_t kUsbBcd = 0x0400;  // distinct from the other XR-DAP boards
constexpr uint16_t kDapPacketSize = 512;
constexpr uint8_t kDapPacketCount = 4;
// DAP_QueueCommands buffer: two packets. The LibXR default (2048) leaves the 20 KB of RAM
// without headroom once the heap and the reserved stack are counted.
constexpr uint16_t kDapQueueBytes = 2 * kDapPacketSize;
// Delay-loop iterations per microsecond of the SWD half period (XR_CLOCK_MODE is the
// system clock in MHz, set by CMake). One iteration takes about 10 CPU cycles, measured
// on hardware from the SWD bit rate at several clock settings in both clock profiles;
// rounded down, every setting ran at or below the requested rate.
constexpr uint32_t kSwdLoopsPerUs = XR_CLOCK_MODE / 10;

constexpr uint8_t PinIndex(uint16_t mask) { return __builtin_ctz(mask); }
// SWCLK and SWDIO are PB13 and PB14 (CubeMX labels SWCLK / SWDIO).
using Swclk = FastGpio<GPIOB_BASE, PinIndex(SWCLK_Pin)>;
using Swdio = FastGpio<GPIOB_BASE, PinIndex(SWDIO_Pin)>;
using Swd = LibXR::Debug::SwdGeneralGPIO<Swclk, Swdio>;
using DapBase = LibXR::USB::DapLinkV2Class<Swd, kDapPacketSize, kDapPacketCount,
                                           kDapPacketSize, kDapQueueBytes>;

// The DAP class, also counting USB bus resets (libxr reports them to every class):
// sampling started from USB stops when the count changes, as the host is gone.
class Dap final : public DapBase
{
 public:
  using DapBase::DapBase;

  static uint32_t BusResets(void* ctx) { return static_cast<Dap*>(ctx)->resets_; }

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
  static Swd swd(swclk, swdio, kSwdLoopsPerUs);

  static Dap dap(LibXR::USB::Endpoint::EPNumber::EP1, LibXR::USB::Endpoint::EPNumber::EP1,
                 swd, &nreset);
  static char serial[SerialBufferSize(kChipUidSize)];
  FormatSerial(ChipUid(), kChipUidSize, serial);
  SetProbeInfo(dap, {kProductName, kBoardName, serial, kFirmwareVersion});
  static DapVendor vendor(swd, kDapPacketSize);
  dap.SetCommandHook(&DapVendor::Hook, &vendor);
  vendor.SetUsbResetSource(&Dap::BusResets, &dap);

  static constexpr auto kStrings = LibXR::USB::DescriptorStrings::MakeLanguagePack(
      LibXR::USB::DescriptorStrings::Language::EN_US, kVendorName, kUsbProductName,
      kSerialPrefix);
  static LibXR::STM32USBDeviceDevFs usb(
      {{ep0_buffer, kFsPacket}, {ep1_dap_out, kFsPacket}},
      {{ep0_buffer, kFsPacket}, {ep1_dap_in, kFsPacket}},
      LibXR::USB::DeviceDescriptor::PacketSize0::SIZE_64, kUsbVid, kUsbPid, kUsbBcd,
      {&kStrings}, {{&dap}}, {ChipUid(), kChipUidSize});
  usb.Init(false);
  usb.Start(false);

  static StatusLed led;
  static Light::StatusLight light(
      static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds()));
  // LibXR runs DAP commands (and the vendor hook) when the application calls Step().
  // Between commands the vendor extension reads a due sampling tick, so Step does not
  // wait while sampling runs.
  for (;;)
  {
    vendor.Poll();
    (void)dap.Step(vendor.Sampling() ? 0 : 1);
    const auto now_ms = static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    const auto link = dap.IsInited() ? Light::Link::WIRED : Light::Link::NONE;
    led.Show(
        light.Update(link, Light::Setting::NONE, vendor.Activity().Read(now_ms), now_ms));
  }
}
