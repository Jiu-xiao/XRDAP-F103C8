// SPDX-License-Identifier: Apache-2.0
#pragma once

// 把状态灯规则算出的 Frame 换成这颗双色灯能做的两件事：亮不亮、红色占多少。纯计算，不碰
// 硬件，test/ 里有 PC 上的测试。
//   亮灭  level 为 0 时熄灭；活动脉冲大于 500 时熄灭（Frame::pulse 对只能亮灭的灯的规定）
//   红色  红色占比（千分比）：1000 全红，0 全蓝；紫色正好是 500，即红蓝各半
//
// Turns a Frame from the status light rule into the two things this two-colour light can
// do: lit or not, and how much red. Pure computation without hardware; host tests are
// in test/.
//   lit   dark when level is 0, and while the activity pulse is above 500 (what
//         Frame::pulse prescribes for an on/off-only LED)
//   red   red share in permille: 1000 all red, 0 all blue; purple is exactly 500, half
//         red and half blue

#include <cstdint>

#include "xrdap/light/status_light.hpp"

namespace XrDap
{

/// 双色灯的一次驱动 / One drive setting of the two-colour light.
struct LedDrive
{
  /// 是否点亮 / Whether the light is on.
  bool lit;
  /// 红色占比，千分比，其余为蓝 / Red share in permille, the rest blue.
  uint32_t red_permille;
};

/// @brief 颜色换成红色占比 / Colour to red share.
///
/// 半占空比在这颗灯上看起来是紫色，所以蓝色分量按 kPurple 的红蓝比折算，kPurple 正好落在
/// 500；既没有红也没有蓝的颜色（彩虹里的绿）取 500。
/// Half duty looks purple on this light, so the blue part is weighted by kPurple's
/// red/blue ratio and kPurple lands exactly on 500; a colour with neither red nor blue
/// (the green of the rainbow) maps to 500.
constexpr uint32_t RedShare(Light::Rgb c)
{
  constexpr Light::Rgb kP = Light::kPurple;
  const uint32_t red = c.r;
  const uint32_t blue = static_cast<uint32_t>(c.b) * kP.r / kP.b;
  return (red + blue == 0U) ? 500U : red * 1000U / (red + blue);
}

/// 一帧换成这颗灯的驱动 / One frame to the drive of this light.
constexpr LedDrive ToLedDrive(const Light::Frame& frame)
{
  return LedDrive{frame.level != 0U && frame.pulse <= 500U, RedShare(frame.color)};
}

}  // namespace XrDap
