// SPDX-License-Identifier: Apache-2.0
#pragma once

// STLINK-V2 板上的红蓝双色灯：两颗 LED 反向并联在 PA9 上，PA9
// 低电平亮蓝、高电平亮红、 高阻两颗都灭。PA9 是 TIM1_CH2 的 PWM
// 输出，占空比就是红色所占的比例，中间值混出紫色； 熄灭时把 PA9 切成浮空输入。
//
// 灯色按 XRDAP-Core 的状态灯规则（status_light.hpp）。本探针只有 USB：USB
// 已配置为有线
// （红），否则为没有连接（紫）。这颗灯不能调亮度，活动脉冲显示为熄灭一下。
//
// Red/blue light of the STLINK-V2 board: two LEDs in anti-parallel on PA9; PA9
// low lights blue, high lights red, high impedance leaves both off. PA9 is the
// TIM1_CH2 PWM output, so the duty cycle is the red share and middle values mix
// purple; to switch the light off PA9 becomes a floating input.
//
// Colours follow XRDAP-Core's status light rule (status_light.hpp). This probe
// has USB only: USB configured is wired (red), otherwise no link (purple). The
// light cannot dim, so an activity pulse shows as a short off blink.

#include <cstdint>

#include "status_light.hpp"

namespace XrDap
{

/// 双色状态灯 / Two-colour status light.
class StatusLed
{
 public:
  /// 在 CubeMX 初始化 TIM1 之后构造：开始 PWM / Construct after CubeMX has set
  /// up TIM1: starts the PWM.
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
