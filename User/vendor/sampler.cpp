// SPDX-License-Identifier: Apache-2.0
#include "sampler.hpp"

#include <cstring>

#include "timebase.hpp"

namespace XrDap::Vendor
{
namespace
{
uint64_t NowUs() { return static_cast<uint64_t>(LibXR::Timebase::GetMicroseconds()); }

void MarkFailed(uint8_t* marks, uint16_t i)
{
  marks[i >> 3] |= static_cast<uint8_t>(1U << (i & 7U));
}
}  // namespace

uint8_t Sampler::Configure(const SubItemList& items, uint32_t period_us, bool stop_on_full,
                           Estimate& est)
{
  uint32_t max_frame = 10U + (items.count + 7U) / 8U;
  uint8_t first_ap = 0U;
  bool first = true;
  items.ForEach(
      [&](const SubItem& sub)
      {
        max_frame += sub.item.size;
        if (first)
        {
          first_ap = sub.item.ap;
          first = false;
        }
      });
  est.max_frame = static_cast<uint16_t>(max_frame);

  // 按当前 SWD 时钟实测：先标定单次访问耗时（INFO 会报告），再把全部条目试读一遍，得到
  // 所有条目都到期时一帧的耗时。
  uint32_t access_us = 0U;
  const bool calibrated = mem_.Calibrate(first_ap, access_us);
  mem_.Release();
  if (!calibrated)
  {
    return kStatusNoTarget;
  }
  access_us_ = access_us;
  uint32_t frame_us = 0U;
  if (!MeasureFrame(items, frame_us))
  {
    return kStatusNoTarget;
  }

  // 乘 1.6：两拍之间主循环还要执行一条 DAP 命令（通常是 SUB_READ），会推迟下一拍；周期
  // 再短就开始跳拍。
  est.frame_us = 10U + (frame_us * 8U) / 5U;
  if (max_frame + 2U > kRingBytes || est.frame_us > period_us)
  {
    return kStatusResource;
  }

  count_ = 0U;
  items.ForEach([&](const SubItem& sub) { items_[count_++] = sub; });
  period_us_ = period_us;
  stop_on_full_ = stop_on_full;
  ring_.Reset();
  configured_ = true;
  return kStatusOk;
}

// 按采样时的访问序列试读两遍，返回第二遍的耗时（第一遍的 CSW 写入等一次性开销不计）。
// 各 AP 的 CSW 先读好，不计入耗时。试读和采样访问同一批地址，副作用相同。
bool Sampler::MeasureFrame(const SubItemList& items, uint32_t& us)
{
  mem_.ResetCache();
  bool ok = true;
  items.ForEach([&](const SubItem& sub)
                { ok = ok && mem_.EnsureAp(sub.item.ap) != nullptr; });
  for (int pass = 0; ok && pass < 2; ++pass)
  {
    MemAp::ApState ap{};
    const uint64_t t0 = NowUs();
    items.ForEach(
        [&](const SubItem& sub)
        {
          const Item& it = sub.item;
          if (ap.ap != it.ap)
          {
            ap = *mem_.EnsureAp(it.ap);
            ap.written = false;
          }
          uint8_t value[8];
          (void)mem_.ReadItem(ap, it, value);
        });
    us = static_cast<uint32_t>(NowUs() - t0);
  }
  mem_.Release();
  return ok;
}

const Sampler::ApCsw* Sampler::FindCsw(uint8_t ap) const
{
  for (uint8_t i = 0; i < csw_count_; ++i)
  {
    if (csw_[i].ap == ap)
    {
      return &csw_[i];
    }
  }
  return nullptr;
}

bool Sampler::Start()
{
  // 开始时读一次各 AP 的 CSW，采样期间不再重读。
  bool ok = true;
  csw_count_ = 0U;
  for (uint16_t i = 0; i < count_; ++i)
  {
    const uint8_t ap = items_[i].item.ap;
    if (FindCsw(ap) != nullptr)
    {
      continue;
    }
    ApCsw& dst = csw_[csw_count_++];
    dst.ap = ap;
    dst.valid = mem_.ReadCsw(ap, dst.base_csw);
    ok = ok && dst.valid;
  }
  mem_.Release();
  if (!ok)
  {
    return false;
  }

  ring_.Reset();
  start_us_ = NowUs();
  tick_ = 0U;
  active_ = true;
  return true;
}

// 读一拍里到期的条目，失败的在 marks 里置位。
void Sampler::SampleTick(const uint16_t* order, uint16_t due, uint8_t* values,
                         uint8_t* marks)
{
  MemAp::ApState ap{};
  for (uint16_t i = 0; i < due; ++i)
  {
    const Item& it = items_[order[i]].item;
    if (ap.ap != it.ap)
    {
      const ApCsw* csw = FindCsw(it.ap);
      ap.ap = it.ap;
      ap.base_csw = (csw != nullptr) ? csw->base_csw : 0U;
      ap.valid = (csw != nullptr) && csw->valid;
      ap.written = false;
    }
    if (mem_.ReadItem(ap, it, values) != kItemOk)
    {
      MarkFailed(marks, i);
    }
    values += it.size;
  }
  mem_.Release();
}

void Sampler::Poll()
{
  if (!active_)
  {
    return;
  }
  const uint64_t tick_us = NowUs();
  if (tick_us < start_us_ + tick_ * period_us_)
  {
    return;
  }

  uint8_t frame[kMaxFrameBytes];
  uint16_t order[kMaxItems];
  uint16_t due = 0U;
  uint16_t values_size = 0U;
  for (uint16_t i = 0; i < count_; ++i)
  {
    if ((tick_ + items_[i].phase) % items_[i].div == 0U)
    {
      order[due++] = i;
      values_size += items_[i].item.size;
    }
  }

  if (due != 0U)
  {
    // 帧：节拍序号 u32 + 时间戳 u48 + 状态位图 + 各到期条目的值。
    const uint16_t bitmap = static_cast<uint16_t>((due + 7U) / 8U);
    const uint16_t base = 10U + bitmap;
    Put32(frame, static_cast<uint32_t>(tick_));
    Put48(frame + 4, tick_us);
    std::memset(frame + 10, 0, bitmap + values_size);
    SampleTick(order, due, frame + base, frame + 10);
    ring_.Push(frame, static_cast<uint16_t>(base + values_size), stop_on_full_);
  }

  // 来不及：跳到下一个还没过去的节拍。
  ++tick_;
  const uint64_t done = NowUs();
  if (done >= start_us_ + tick_ * period_us_)
  {
    const uint64_t next = (done - start_us_) / period_us_ + 1U;
    ring_.AddSkipped(static_cast<uint32_t>(next - tick_));
    tick_ = next;
  }
}

}  // namespace XrDap::Vendor
