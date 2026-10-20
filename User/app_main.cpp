// SPDX-License-Identifier: Apache-2.0
// XR-DAP-F103: a CMSIS-DAP v2 debug probe on STLINK-V2 style STM32F103 boards, built on
// LibXR. One USB full-speed Bulk v2 DAP interface with the XR vendor extension
// (multi-address access, pointer chains, probe-side sampling); the LED on PA9 shows DAP
// activity.
//
// XR-DAP-F103：基于 LibXR、跑在 STLINK-V2 形态 STM32F103 板子上的 CMSIS-DAP v2 调试探针。
// 一个 USB 全速 Bulk v2 DAP 接口，带 XR 厂商扩展（多地址读写、指针链、探针端采样）；
// PA9 上的指示灯显示 DAP 活动。

#include "app_main.h"

#include <cstdint>

#include "dap_vendor.hpp"
#include "daplink_v2_profile_swd.hpp"
#include "debug/swd_general_gpio.hpp"
#include "fast_gpio.hpp"
#include "libxr.hpp"
#include "main.h"
#include "probe_info.hpp"
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
// Delay-loop iterations per microsecond of the SWD half period. One iteration takes
// about 10 CPU cycles, so the value scales with the system clock (XR_CLOCK_MODE in MHz);
// rounded down, every SWD clock setting runs at or below the requested rate.
constexpr uint32_t kSwdLoopsPerUs = XR_CLOCK_MODE / 10;

constexpr uint8_t PinIndex(uint16_t mask) { return __builtin_ctz(mask); }
// SWCLK and SWDIO are PB13 and PB14 (CubeMX labels SWCLK / SWDIO).
using Swclk = FastGpio<GPIOB_BASE, PinIndex(SWCLK_Pin)>;
using Swdio = FastGpio<GPIOB_BASE, PinIndex(SWDIO_Pin)>;
using Swd = LibXR::Debug::SwdGeneralGPIO<Swclk, Swdio>;
using Dap = LibXR::USB::DapLinkV2Class<Swd, kDapPacketSize, kDapPacketCount, kDapPacketSize>;

// FSDEV packet memory is 512 B: 8 B of BTABLE per endpoint, then 64 B per direction.
// LibXR endpoint slot memory must be aligned to HW_CACHE_LINE_SIZE.
constexpr size_t kFsPacket = 64;
alignas(LibXR::HW_CACHE_LINE_SIZE) uint8_t ep0_buffer[kFsPacket];
alignas(LibXR::HW_CACHE_LINE_SIZE) uint8_t ep1_dap_in[2 * kDapPacketSize];
alignas(LibXR::HW_CACHE_LINE_SIZE) uint8_t ep1_dap_out[2 * kDapPacketSize];

// Activity LED: TIM1 CH2 PWM (CubeMX sets ARR = 0xFFFF); the duty follows a low-pass
// filter of "DAP IN endpoint busy".
void UpdateActivityLed(bool busy)
{
  static uint32_t level = 0;
  constexpr uint32_t kShift = 8;  // time constant ~256 loop passes
  const uint32_t target = busy ? 0xFFFFU : 0U;
  level = level - (level >> kShift) + (target >> kShift);
  TIM1->CCR2 = level;
}
}  // namespace

extern TIM_HandleTypeDef htim1;

extern "C" void app_main(void)
{
  static LibXR::STM32Timebase timebase;
  LibXR::PlatformInit();

  static Swclk swclk;
  static Swdio swdio;
  static LibXR::STM32GPIO nreset(NRST_GPIO_Port, NRST_Pin);
  nreset.Write(true);
  static Swd swd(swclk, swdio, kSwdLoopsPerUs);

  static Dap dap(LibXR::USB::Endpoint::EPNumber::EP1, LibXR::USB::Endpoint::EPNumber::EP1,
                 swd, &nreset);
  SetProbeInfo(dap);
  static DapVendor vendor(swd, kDapPacketSize);
  dap.SetCommandHook(&DapVendor::Hook, &vendor);

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

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  // LibXR runs DAP commands (and the vendor hook) when the application calls Step().
  // Between commands the vendor extension reads a due sampling tick, so Step does not
  // wait while sampling runs.
  for (;;)
  {
    vendor.Poll(dap.IsInited());
    (void)dap.Step(vendor.Sampling() ? 0 : 1);
    UpdateActivityLed(dap.EpInBusy());
  }
}
