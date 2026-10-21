// SPDX-License-Identifier: Apache-2.0
#include "tick_timer.hpp"

#include "main.h"

namespace
{
XrDap::Tim2TickTimer* instance = nullptr;

constexpr uint32_t kMaxCount = 0x10000U;  // TIM2 是 16 位 / TIM2 is 16 bits
constexpr uint32_t kIrqPriority = 1U;     // USB 中断是 0 / The USB interrupts are 0

// TIM2 在 APB1 上；APB1 分频不为 1 时定时器时钟是 PCLK1 的两倍（参考手册时钟树）/ TIM2
// is on APB1; when APB1 is divided, the timer clock is twice PCLK1 (reference manual
// clock tree).
uint32_t TimerClockHz()
{
  const uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  return ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1) ? pclk1 : 2U * pclk1;
}
}  // namespace

extern "C" void TIM2_IRQHandler(void)
{
  if (instance != nullptr)
  {
    instance->OnUpdateIsr();
  }
}

namespace XrDap
{

Tim2TickTimer::Tim2TickTimer(DapVendor& vendor) : vendor_(vendor)
{
  instance = this;
  __HAL_RCC_TIM2_CLK_ENABLE();
  // 只有计数溢出产生更新中断 / Only overflow raises the update interrupt.
  TIM2->CR1 = TIM_CR1_URS;
  TIM2->PSC = TimerClockHz() / 1000000U - 1U;
  HAL_NVIC_SetPriority(TIM2_IRQn, kIrqPriority, 0U);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void Tim2TickTimer::Start(uint32_t period_us)
{
  Stop();
  if (period_us == 0U)
  {
    return;
  }
  ticks_per_irq_ =
      (period_us < kMinIrqUs) ? (kMinIrqUs + period_us - 1U) / period_us : 1U;
  const uint64_t irq_us = static_cast<uint64_t>(period_us) * ticks_per_irq_;
  segments_ = static_cast<uint32_t>((irq_us + kMaxCount - 1U) / kMaxCount);
  segment_us_ = static_cast<uint32_t>(irq_us / segments_);
  longer_segments_ = static_cast<uint32_t>(irq_us % segments_);
  segment_ = 0U;

  TIM2->ARR = SegmentUs(0U) - 1U;
  TIM2->CNT = 0U;
  // 装入 PSC；URS 置位，不产生中断 / Loads PSC; URS is set, so no interrupt.
  TIM2->EGR = TIM_EGR_UG;
  TIM2->SR = 0U;
  TIM2->DIER = TIM_DIER_UIE;
  TIM2->CR1 |= TIM_CR1_CEN;
}

void Tim2TickTimer::Stop()
{
  TIM2->CR1 &= ~TIM_CR1_CEN;
  TIM2->DIER = 0U;
  TIM2->SR = 0U;
  HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
}

// 第 index 段的长度（us）：前 longer_segments_ 段多 1 us / Length of segment index in
// us: the first longer_segments_ segments are 1 us longer.
uint32_t Tim2TickTimer::SegmentUs(uint32_t index) const
{
  return segment_us_ + ((index < longer_segments_) ? 1U : 0U);
}

void Tim2TickTimer::OnUpdateIsr()
{
  TIM2->SR = ~TIM_SR_UIF;
  if (segments_ > 1U)
  {
    segment_ = (segment_ + 1U == segments_) ? 0U : segment_ + 1U;
    // ARR 没有预装载，写入对刚开始的这一段立即生效；计数器刚从 0 开始，段长至少 32768
    // us，不会已经越过新值 / ARR is not preloaded, so the write applies to the segment
    // that just started; the counter has just restarted from 0 and a segment is at
    // least 32768 us, so it cannot be past the new value.
    TIM2->ARR = SegmentUs(segment_) - 1U;
    if (segment_ != 0U)
    {
      return;
    }
  }
  for (uint32_t i = 0U; i < ticks_per_irq_; ++i)
  {
    vendor_.OnTickIsr();
  }
}

}  // namespace XrDap
