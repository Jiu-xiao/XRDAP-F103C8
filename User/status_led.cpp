// SPDX-License-Identifier: Apache-2.0
#include "status_led.hpp"

#include "led_drive.hpp"
#include "main.h"

extern TIM_HandleTypeDef htim1;

namespace
{
// PA9 在 GPIOA->CRH 中的 4 位模式字段 / The 4-bit mode field of PA9 in GPIOA->CRH.
constexpr uint32_t kPinShift = (9U - 8U) * 4U;
constexpr uint32_t kPinMask = 0xFU << kPinShift;
constexpr uint32_t kFloatingInput = 0x4U << kPinShift;
}  // namespace

namespace XrDap
{

StatusLed::StatusLed()
{
  lit_mode_ = GPIOA->CRH & kPinMask;  // CubeMX: TIM1_CH2 复用推挽 / alternate push-pull
  SetRed(0U);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

void StatusLed::Show(const Light::Frame& frame)
{
  const LedDrive drive = ToLedDrive(frame);
  SetLit(drive.lit);
  SetRed(drive.red_permille);
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
