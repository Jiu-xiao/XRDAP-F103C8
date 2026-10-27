// SPDX-License-Identifier: Apache-2.0
#pragma once

// 采样的节拍定时器（XRDAP-Core 的 TickTimer），用 TIM2。TIM2 以 1 MHz 计数，更新中断里
// 调用 DapVendor::OnTickIsr()。CubeMX 没有配置 TIM2，时钟、中断和寄存器都在这里设置。
//   周期超过 16 位计数（65536 us）时，一个周期拆成若干段，段长相差不超过 1 us，最后一段
//   结束时记一拍。
//   周期短于 kMinIrqUs 时，每 m 个周期中断一次、记 m 拍，避免中断占满 CPU；这么短的
//   周期本来就采不过来，采样按跳拍处理。
// Start 和 Stop 在主循环里调用（执行上下文），中断只计数并在该采时叫醒主循环。
//
// Sampling tick timer (XRDAP-Core's TickTimer) on TIM2. TIM2 counts at 1 MHz and its
// update interrupt calls DapVendor::OnTickIsr(). CubeMX does not configure TIM2, so its
// clock, interrupt and registers are set up here.
//   A period longer than the 16-bit count (65536 us) is split into segments that differ
//   by at most 1 us; the end of the last segment counts one tick.
//   A period shorter than kMinIrqUs interrupts once every m periods and counts m ticks,
//   so the interrupt cannot take the whole CPU; such periods cannot be sampled anyway
//   and sampling counts the ticks as skipped.
// Start and Stop are called from the main loop (the execution context); the interrupt
// only counts and wakes the main loop when a tick is due.

#include <cstdint>

#include "xrdap/vendor/dap_vendor.hpp"

namespace XrDap
{

class Tim2TickTimer final : public Vendor::TickTimer
{
 public:
  /// 最短的中断间隔 / Shortest interrupt interval.
  static constexpr uint32_t kMinIrqUs = 50U;

  /// @note 只能有一个实例，初始化时构造 / A single instance, constructed at
  ///       initialisation.
  explicit Tim2TickTimer(DapVendor& vendor);

  void Start(uint32_t period_us) override;
  void Stop() override;

  /// TIM2 中断调用 / Called by the TIM2 interrupt.
  void OnUpdateIsr();

 private:
  uint32_t SegmentUs(uint32_t index) const;

  DapVendor& vendor_;
  // Start 写、中断读；Start 写之前定时器已停 / Written by Start, read by the interrupt;
  // the timer is stopped while Start writes them.
  uint32_t ticks_per_irq_ = 1U;
  uint32_t segments_ = 1U;
  uint32_t segment_us_ = 0U;
  uint32_t longer_segments_ = 0U;
  uint32_t segment_ = 0U;
};

}  // namespace XrDap
