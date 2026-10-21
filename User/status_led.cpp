// SPDX-License-Identifier: Apache-2.0
#include "status_led.hpp"

#include "main.h"
#include "timebase.hpp"

extern TIM_HandleTypeDef htim1;

namespace
{
/// 上电扫色的时间 / Duration of the power-up sweep.
constexpr uint32_t kBootSweepMs = 800U;
/// 调试器已连接时红紫往返、目标运行中时红蓝往返一次的时间 / One red-purple round while
/// a debugger is connected, one red-blue round while the target runs.
constexpr uint32_t kConnectedPeriodMs = 2000U;
constexpr uint32_t kRunPeriodMs = 1000U;
/// 红色所占的比例（千分比）：全红、紫（红蓝各半）/ Red share (permille): full red,
/// purple (half red, half blue).
constexpr uint32_t kRed = 1000U;
constexpr uint32_t kPurple = 500U;
/// 活动闪灭：熄灭的时间，以及两次熄灭的最短间隔（约 15 次每秒）/ Activity blink: how
/// long the light is off, and the shortest interval between two blinks (about 15 per
/// second).
constexpr uint32_t kBlinkOffMs = 30U;
constexpr uint32_t kBlinkPeriodMs = 66U;

// PA9 在 GPIOA->CRH 中的 4 位模式字段 / The 4-bit mode field of PA9 in GPIOA->CRH.
constexpr uint32_t kPinShift = (9U - 8U) * 4U;
constexpr uint32_t kPinMask = 0xFU << kPinShift;
constexpr uint32_t kFloatingInput = 0x4U << kPinShift;

uint32_t NowMs() { return static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds()); }

// 三角波，千分比：0 升到 1000 再回到 0，用时 period_ms / Triangle wave in permille:
// 0 up to 1000 and back to 0 within period_ms.
uint32_t Triangle(uint32_t ms, uint32_t period_ms)
{
  const uint32_t phase = ms % period_ms;
  const uint32_t half = period_ms / 2U;
  const uint32_t up = (phase < half) ? phase : period_ms - phase;
  return up * 1000U / half;
}
}  // namespace

namespace XrDap
{

StatusLed::StatusLed() : start_ms_(NowMs()), blink_ms_(start_ms_ - kBlinkPeriodMs)
{
  lit_mode_ = GPIOA->CRH & kPinMask;  // CubeMX: TIM1_CH2 复用推挽 / alternate push-pull
  SetRed(0U);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

void StatusLed::Update(bool usb_up, const Vendor::DapActivity::State& dap,
                       uint32_t now_ms)
{
  const uint32_t t = now_ms - start_ms_;
  if (dap.commands != seen_commands_)
  {
    seen_commands_ = dap.commands;
    blink_pending_ = true;
  }
  if (t < kBootSweepMs)
  {
    SetLit(true);
    SetRed(Triangle(t, kBootSweepMs));
    return;
  }

  // 有新命令且距上次熄灭够久时再熄灭一次；命令不断时按 kBlinkPeriodMs 稳定闪烁。
  // A new command blinks again once the previous blink is old enough; steady traffic
  // blinks at a steady kBlinkPeriodMs.
  if (blink_pending_ && now_ms - blink_ms_ >= kBlinkPeriodMs)
  {
    blink_ms_ = now_ms;
    blink_pending_ = false;
  }
  SetLit(now_ms - blink_ms_ >= kBlinkOffMs);

  if (!usb_up)
  {
    SetRed(kPurple);
  }
  else if (dap.running)
  {
    SetRed(kRed - Triangle(t, kRunPeriodMs));  // 红 → 蓝 → 红 / red -> blue -> red
  }
  else if (dap.connected)
  {
    SetRed(kRed - (kRed - kPurple) * Triangle(t, kConnectedPeriodMs) / 1000U);
  }
  else
  {
    SetRed(kRed);
  }
}

// TIM1 由 CubeMX 设为 ARR = 0xFFFF，PWM1、高电平有效：CCR2 是一个周期里 PA9 为高（红）
// 的计数。CCR2 是 16 位，满量程取 ARR。
// CubeMX sets TIM1 to ARR = 0xFFFF, PWM1, active high: CCR2 counts the part of a period
// with PA9 high (red). CCR2 is 16 bits, so full scale is ARR.
void StatusLed::SetRed(uint32_t permille)
{
  const uint32_t top = TIM1->ARR;
  const uint32_t ccr = (top + 1U) * permille / 1000U;
  TIM1->CCR2 = (ccr > top) ? top : ccr;
}

void StatusLed::SetLit(bool lit)
{
  if (lit == lit_)
  {
    return;
  }
  lit_ = lit;
  GPIOA->CRH = (GPIOA->CRH & ~kPinMask) | (lit ? lit_mode_ : kFloatingInput);
}

}  // namespace XrDap
