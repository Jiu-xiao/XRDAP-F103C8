// SPDX-License-Identifier: Apache-2.0
#pragma once

// XRDAP-Core 的平台部分：采样帧缓冲用的自旋锁。本探针的采样和 DAP 命令都在主循环里执行，
// 从不并发，锁是空操作。
//
// Platform part of XRDAP-Core: the spinlock of the sample frame buffer. Sampling and DAP
// commands both run in this probe's main loop and never overlap, so the lock does
// nothing.

namespace XrDap::Port
{

/// 空锁 / No-op lock.
class SpinLock
{
 public:
  void Lock() {}
  void Unlock() {}
};

}  // namespace XrDap::Port
