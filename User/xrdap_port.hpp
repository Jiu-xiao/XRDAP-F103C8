// SPDX-License-Identifier: Apache-2.0
#pragma once

// XRDAP-Core 的平台部分：采样帧缓冲的大小和它的自旋锁。本探针的采样和 DAP 命令都在主循
// 环里执行，从不并发，锁是空操作。
//
// Platform part of XRDAP-Core: the size of the sample frame buffer and its spinlock.
// Sampling and DAP commands both run in this probe's main loop and never overlap, so the
// lock does nothing.

#include <cstdint>

namespace XrDap::Vendor
{
class LoopSampling;
}  // namespace XrDap::Vendor

namespace XrDap::Session
{
class SingleTransport;
}  // namespace XrDap::Session

namespace XrDap::Port
{
/// 单循环采样：主循环在两条命令之间调用 Poll / Single-loop sampling: the main loop calls Poll
/// between commands.
using SamplerPolicy = Vendor::LoopSampling;

/// 只有 USB 一条传输 / USB is the only transport.
using TransportPolicy = Session::SingleTransport;


/// 采样帧缓冲的字节数，2 的幂 / Bytes of the sample frame buffer, a power of two.
inline constexpr uint32_t kRingBytes = 4096U;

/// 空锁 / No-op lock.
class SpinLock
{
 public:
  void Lock() {}
  void Unlock() {}
};

}  // namespace XrDap::Port
