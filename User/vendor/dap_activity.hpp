// SPDX-License-Identifier: Apache-2.0
#pragma once

// DAP 活动和上位机状态：数每条 DAP 命令，记下 DAP_HostStatus 报告的“已连接”“运行中”，
// 给状态灯用。DAP_Disconnect 清除这两个状态；上位机异常退出没发 Disconnect 时，
// kHostQuietMs 内没有任何命令也视为清除。命令和状态灯都在主循环里，不需要原子量。
//
// DAP activity and host state: counts every DAP command and keeps the "connected" and
// "running" flags reported by DAP_HostStatus, for the status light. DAP_Disconnect clears
// both; when a host dies without a Disconnect, kHostQuietMs without any command clears
// them as well. Commands and the status light both run in the main loop, so no atomics.

#include <cstdint>

namespace XrDap::Vendor
{

/// DAP 活动统计 / DAP activity.
class DapActivity
{
 public:
  /// 多久收不到命令就认为上位机已离开 / How long without commands before the host is
  /// taken as gone.
  static constexpr uint32_t kHostQuietMs = 5000U;

  /// 某一时刻的状态 / State at one moment.
  struct State
  {
    /// 累计命令数，只看变化 / Running command count; only changes matter.
    uint32_t commands;
    /// 上位机报告已连接 / Host reported connected.
    bool connected;
    /// 上位机报告目标运行中 / Host reported the target running.
    bool running;
  };

  /// 每条 DAP 命令调用一次 / Called once for every DAP command.
  void Observe(const uint8_t* req, uint16_t len, uint32_t now_ms)
  {
    last_ms_ = now_ms;
    ++commands_;
    if (req[0] == kHostStatus && len >= 3U)
    {
      // DAP_HostStatus：类型 0 是“已连接”灯，1 是“运行中”灯；状态 1 亮、0 灭
      (req[1] == 0U ? connected_ : running_) = (req[2] != 0U);
    }
    else if (req[0] == kDisconnect)
    {
      connected_ = false;
      running_ = false;
    }
  }

  /// 读当前状态 / Read the current state.
  State Read(uint32_t now_ms) const
  {
    const bool quiet = now_ms - last_ms_ > kHostQuietMs;
    return State{commands_, !quiet && connected_, !quiet && running_};
  }

 private:
  static constexpr uint8_t kHostStatus = 0x01U;
  static constexpr uint8_t kDisconnect = 0x03U;

  uint32_t commands_ = 0U;
  uint32_t last_ms_ = 0U;
  bool connected_ = false;
  bool running_ = false;
};

}  // namespace XrDap::Vendor
