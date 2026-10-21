// SPDX-License-Identifier: Apache-2.0
#pragma once

// STLINK-V2 板上的红蓝双色灯。两颗 LED 反向并联在 PA9 上：PA9 低电平亮蓝，高电平亮红，
// 高阻时两颗都灭。PA9 是 TIM1_CH2 的 PWM 输出，占空比就是红色所占的比例，中间值混出紫色；
// 熄灭时把 PA9 切成浮空输入。灯色由 XRDAP-Core 的状态灯规则算出（status_light.hpp），
// 换算见 led_drive.hpp。
//
// 只在主循环里使用：构造和 Show() 都不可在中断里调用。它独占 TIM1 的 CCR2，并对 GPIOA->CRH
// 做读-改-写；这个寄存器也管着 PA8 到 PA15（包括 USB 的 PA11、PA12 和本芯片的调试口
// PA13、PA14）。初始化之后不再有别的代码写 GPIOA->CRH（USB 外设接管引脚不经过它），所以
// 这样改是安全的；要在别处改 GPIOA 高 8 位引脚的模式，需要和这里一起考虑。
//
// Red/blue light of the STLINK-V2 board. Two LEDs in anti-parallel on PA9: PA9 low lights
// blue, high lights red, high impedance leaves both off. PA9 is the TIM1_CH2 PWM output,
// so the duty cycle is the red share and middle values mix purple; to switch the light
// off PA9 becomes a floating input. The colours come from XRDAP-Core's status light rule
// (status_light.hpp); the conversion is in led_drive.hpp.
//
// Main loop only: neither the constructor nor Show() may run in an interrupt. It owns
// TIM1 CCR2 and does a read-modify-write of GPIOA->CRH, which also holds PA8 to PA15
// (including the USB pins PA11 and PA12 and this chip's debug port PA13 and PA14). Nothing
// else writes GPIOA->CRH after initialisation (the USB peripheral takes its pins without
// it), so this is safe; changing the mode of another upper GPIOA pin elsewhere has to be
// considered together with this class.

#include <cstdint>

#include "status_light.hpp"

namespace XrDap
{

/// 双色状态灯 / Two-colour status light.
class StatusLed
{
 public:
  /// 在 CubeMX 初始化 TIM1 之后构造：开始 PWM / Construct after CubeMX has set up TIM1:
  /// starts the PWM.
  StatusLed();

  /// 显示规则算出的一帧 / Show one frame computed by the rule.
  void Show(const Light::Frame& frame);

 private:
  void SetRed(uint32_t permille);
  void SetLit(bool lit);

  bool lit_ = true;
  uint32_t lit_mode_ = 0U;  ///< PA9 的 PWM 输出模式位（CRH）/ PA9 PWM mode bits (CRH)
};

}  // namespace XrDap
