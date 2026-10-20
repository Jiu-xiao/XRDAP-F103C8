// SPDX-License-Identifier: Apache-2.0
#pragma once

// 采样帧的环形缓冲。每帧前面有 2 字节小端长度，只按整帧取出。采样和 SUB_READ 都在主循环
// 里执行，不需要锁。
//
// Ring buffer of sample frames. Each frame is preceded by a 2-byte little-endian length
// and is only taken out whole. Sampling and SUB_READ both run in the main loop, so no
// lock is needed.

#include <cstdint>

#include "vendor_protocol.hpp"

namespace XrDap::Vendor
{

/// 采样帧环形缓冲 / Ring buffer of sample frames.
class FrameRing
{
 public:
  /// 一次取帧的结果 / Result of one take.
  struct Taken
  {
    /// 拷出的字节数 / Bytes copied out.
    uint32_t bytes = 0;
    /// 拷出的帧数 / Frames copied out.
    uint16_t frames = 0;
    /// 缓冲里剩下的帧数据字节数（不含长度头）/ Frame bytes left (length headers
    /// excluded).
    uint32_t remaining = 0;
    /// 累计丢弃的帧数 / Frames dropped so far.
    uint32_t dropped = 0;
    /// 累计跳过的节拍数 / Ticks skipped so far.
    uint32_t skipped = 0;
  };

  /// 清空缓冲和计数 / Clear the buffer and the counters.
  void Reset();
  /// @brief 放入一帧 / Put one frame in.
  /// @param stop_on_full 缓冲满时丢新帧；否则丢最旧的帧 / When full, drop the new frame;
  ///        otherwise drop the oldest frames.
  void Push(const uint8_t* frame, uint16_t len, bool stop_on_full);
  /// 取出不超过 limit 字节的整帧，帧之间不留长度头 / Take whole frames, at most limit
  /// bytes, without the length headers.
  Taken Take(uint8_t* dst, uint32_t limit);
  /// 记下跳过的节拍 / Count skipped ticks.
  void AddSkipped(uint32_t n);

 private:
  static_assert((kRingBytes & (kRingBytes - 1U)) == 0U, "kRingBytes 必须是 2 的幂");

  uint16_t FrameLen(uint32_t pos) const;
  void CopyIn(uint32_t at, const uint8_t* src, uint32_t len);
  void CopyOut(uint32_t at, uint8_t* dst, uint32_t len) const;

  uint8_t buf_[kRingBytes] = {};
  uint32_t head_ = 0U;
  uint32_t tail_ = 0U;
  uint32_t used_ = 0U;
  uint32_t payload_ = 0U;
  uint32_t dropped_ = 0U;
  uint32_t skipped_ = 0U;
};

}  // namespace XrDap::Vendor
