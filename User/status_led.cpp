// SPDX-License-Identifier: Apache-2.0
#include "status_led.hpp"

#include "main.h"

extern TIM_HandleTypeDef htim1;

namespace
{
// PA9 在 GPIOA->CRH 中的 4 位模式字段 / The 4-bit mode field of PA9 in
// GPIOA->CRH.
constexpr uint32_t kPinShift = (9U - 8U) * 4U;
constexpr uint32_t kPinMask = 0xFU << kPinShift;
constexpr uint32_t kFloatingInput = 0x4U << kPinShift;

// 颜色换成红色所占的比例（千分比）。半占空比在这颗灯上看起来是紫色，所以蓝色分量按
// kPurple 的红蓝比折算，kPurple 正好落在 500。
// Colour to red share (permille). Half duty looks purple on this light, so the
// blue part is weighted by kPurple's red/blue ratio and kPurple lands exactly
// on 500.
uint32_t RedShare(XrDap::Light::Rgb c)
{
  constexpr XrDap::Light::Rgb kP = XrDap::Light::kPurple;
  const uint32_t red = c.r;
  const uint32_t blue = c.b * kP.r / kP.b;
  return (red + blue == 0U) ? 500U : red * 1000U / (red + blue);
}
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
  SetLit(frame.level != 0U && frame.pulse <= 500U);
  SetRed(RedShare(frame.color));
}

// TIM1 由 CubeMX 设为 ARR = 0xFFFF，PWM1、高电平有效：CCR2 是一个周期里 PA9
// 为高（红） 的计数。CCR2 是 16 位，满量程取 ARR。 CubeMX sets TIM1 to ARR =
// 0xFFFF, PWM1, active high: CCR2 counts the part of a period with PA9 high
// (red). CCR2 is 16 bits, so full scale is ARR.
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
