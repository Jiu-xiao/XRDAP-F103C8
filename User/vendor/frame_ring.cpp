// SPDX-License-Identifier: Apache-2.0
#include "frame_ring.hpp"

#include <cstring>

namespace XrDap::Vendor
{
namespace
{
constexpr uint32_t kMask = kRingBytes - 1U;
}  // namespace

void FrameRing::Reset()
{
  head_ = 0U;
  tail_ = 0U;
  used_ = 0U;
  payload_ = 0U;
  dropped_ = 0U;
  skipped_ = 0U;
}

uint16_t FrameRing::FrameLen(uint32_t pos) const
{
  return static_cast<uint16_t>(buf_[pos] | (buf_[(pos + 1U) & kMask] << 8));
}

void FrameRing::CopyIn(uint32_t at, const uint8_t* src, uint32_t len)
{
  const uint32_t first = (at + len <= kRingBytes) ? len : kRingBytes - at;
  std::memcpy(buf_ + at, src, first);
  std::memcpy(buf_, src + first, len - first);
}

void FrameRing::CopyOut(uint32_t at, uint8_t* dst, uint32_t len) const
{
  const uint32_t first = (at + len <= kRingBytes) ? len : kRingBytes - at;
  std::memcpy(dst, buf_ + at, first);
  std::memcpy(dst + first, buf_, len - first);
}

void FrameRing::Push(const uint8_t* frame, uint16_t len, bool stop_on_full)
{
  while (used_ + len + 2U > kRingBytes)
  {
    if (stop_on_full || used_ == 0U)
    {
      ++dropped_;
      return;
    }
    const uint16_t old = FrameLen(tail_);
    tail_ = (tail_ + 2U + old) & kMask;
    used_ -= 2U + old;
    payload_ -= old;
    ++dropped_;
  }
  const uint8_t hdr[2] = {static_cast<uint8_t>(len), static_cast<uint8_t>(len >> 8)};
  CopyIn(head_, hdr, 2U);
  CopyIn((head_ + 2U) & kMask, frame, len);
  head_ = (head_ + 2U + len) & kMask;
  used_ += 2U + len;
  payload_ += len;
}

FrameRing::Taken FrameRing::Take(uint8_t* dst, uint32_t limit)
{
  Taken t;
  uint32_t pos = tail_;
  while (used_ != 0U)
  {
    const uint16_t flen = FrameLen(pos);
    if (t.bytes + flen > limit)
    {
      break;
    }
    CopyOut((pos + 2U) & kMask, dst + t.bytes, flen);
    pos = (pos + 2U + flen) & kMask;
    t.bytes += flen;
    ++t.frames;
    used_ -= 2U + flen;
    payload_ -= flen;
  }
  tail_ = pos;
  t.remaining = payload_;
  t.dropped = dropped_;
  t.skipped = skipped_;
  return t;
}

void FrameRing::AddSkipped(uint32_t n)
{
  skipped_ += n;
}

}  // namespace XrDap::Vendor
