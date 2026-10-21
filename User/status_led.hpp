// SPDX-License-Identifier: Apache-2.0
#pragma once

// STLINK-V2 板上的红蓝双色灯：两颗 LED 反向并联在 PA9 上，PA9 低电平亮蓝、高电平亮红、
// 高阻两颗都灭。PA9 是 TIM1_CH2 的 PWM 输出，占空比就是红色所占的比例，中间值混出紫色；
// 熄灭时把 PA9 切成浮空输入。
//
// XR-DAP 探针的灯色约定：红色是有线（USB）连接，蓝色是无线连接，紫色是没有连接或正在改
// 配置。本探针只有 USB：
//   上电          蓝 → 红 → 蓝扫一次
//   USB 未配置    紫色常亮（还没有电脑配置这个设备，例如只接了电源）
//   USB 空闲      红色常亮
//   调试器已连接  红、紫之间往返，2 秒一次（DAP_HostStatus 的“已连接”）
//   目标运行中    红、蓝之间往返，1 秒一次（DAP_HostStatus 的“运行中”）
//   每条 DAP 命令  熄灭一下，可重复触发，最快约 15 次每秒
//
// Red/blue light of the STLINK-V2 board: two LEDs in anti-parallel on PA9; PA9 low lights
// blue, high lights red, high impedance leaves both off. PA9 is the TIM1_CH2 PWM output,
// so the duty cycle is the red share and middle values mix purple; to switch the light
// off PA9 becomes a floating input.
//
// XR-DAP light colours: red is a wired (USB) link, blue a wireless link, purple no link or
// a configuration change. This probe has USB only:
//   power-up            one sweep blue -> red -> blue
//   USB not configured  solid purple (no computer has configured the device yet, for
//                       example on a power supply only)
//   USB idle            solid red
//   debugger connected  red and purple back and forth every 2 s (DAP_HostStatus
//                       "connected")
//   target running      red and blue back and forth every second (DAP_HostStatus
//                       "running")
//   every DAP command   a short off blink, retriggered, at most about 15 per second

#include <cstdint>

#include "dap_activity.hpp"

namespace XrDap
{

/// 双色状态灯 / Two-colour status light.
class StatusLed
{
 public:
  /// 在 CubeMX 初始化 TIM1 之后构造：开始 PWM / Construct after CubeMX has set up TIM1:
  /// starts the PWM.
  StatusLed();

  /// @brief 主循环每圈调用 / Called on every main-loop pass.
  /// @param usb_up DAP 接口已被主机配置 / The DAP interface is configured by the host.
  void Update(bool usb_up, const Vendor::DapActivity::State& dap, uint32_t now_ms);

 private:
  void SetRed(uint32_t permille);
  void SetLit(bool lit);

  uint32_t start_ms_;
  uint32_t blink_ms_;  ///< 最近一次熄灭的开始时刻 / Start of the latest blink
  uint32_t seen_commands_ = 0U;
  bool blink_pending_ = false;
  bool lit_ = true;
  uint32_t lit_mode_ = 0U;  ///< PA9 的 PWM 输出模式位（CRH）/ PA9 PWM mode bits (CRH)
};

}  // namespace XrDap
