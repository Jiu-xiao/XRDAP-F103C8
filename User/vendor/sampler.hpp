// SPDX-License-Identifier: Apache-2.0
#pragma once

// 探针端周期采样。第 k 拍的目标时刻为 start + k × period，不累计漂移；来不及时跳过节拍，
// 不补做。没有任务也没有定时器中断：主循环在两条 DAP 命令之间调用 Poll()，到期就读这一拍。
// 一拍读完必定回到主循环处理一条命令，所以采样不会饿死 DAP 命令（例如 SUB_READ、停止）。
//
// Probe-side periodic sampling. Tick k is due at start + k × period with no accumulated
// drift; a late tick is skipped, not made up. There is no task and no timer interrupt:
// the main loop calls Poll() between DAP commands and reads the tick once it is due.
// After every tick control returns to the main loop, which handles one command, so
// sampling never starves DAP commands (for example SUB_READ or a stop).

#include <cstdint>

#include "frame_ring.hpp"
#include "mem_ap.hpp"
#include "vendor_protocol.hpp"

namespace XrDap::Vendor
{

/// 采样配置、节拍调度和帧缓冲 / Sampling configuration, tick scheduling and frame buffer.
class Sampler
{
 public:
  /// SUB_SET 的估计结果 / Estimate reported by SUB_SET.
  struct Estimate
  {
    /// 一帧的最大字节数 / Maximum bytes of one frame.
    uint16_t max_frame = 0;
    /// 估计的单帧耗时（微秒）/ Estimated frame time in microseconds.
    uint32_t frame_us = 0;
  };

  explicit Sampler(MemAp& mem) : mem_(mem) {}

  /// @brief 实测这组条目一帧的耗时并保存配置（SUB_SET）/ Measure the frame time of these
  ///        items and store the configuration (SUB_SET).
  /// @return kStatusOk；估计耗时超过周期或帧太大时 kStatusResource；SWD 访问失败时
  ///         kStatusNoTarget / kStatusOk; kStatusResource when the estimate exceeds the
  ///         period or the frame is too big; kStatusNoTarget when SWD access fails.
  uint8_t Configure(const SubItem* items, uint16_t count, uint32_t period_us,
                    bool stop_on_full, Estimate& est);
  /// 是否有有效配置 / Whether a valid configuration exists.
  bool Configured() const { return configured_; }
  /// 是否正在采样 / Whether sampling is running.
  bool Active() const { return active_; }
  /// 最近一次标定的单次访问耗时（微秒），0 表示还没有标定 / Last calibrated per-access
  /// time in microseconds; 0 before any calibration.
  uint32_t AccessUs() const { return access_us_; }

  /// 读好各 AP 的 CSW 后开始采样；失败时返回 false / Read every AP's CSW, then start;
  /// false on failure.
  bool Start();
  /// 停止采样 / Stop sampling.
  void Stop() { active_ = false; }
  /// 主循环每圈调用：节拍到期时读一拍 / Called on every main-loop pass: reads one tick
  /// when it is due.
  void Poll();
  /// 清空帧缓冲和计数 / Clear the frame buffer and the counters.
  void ClearFrames() { ring_.Reset(); }
  /// 取出整帧（SUB_READ）/ Take whole frames (SUB_READ).
  FrameRing::Taken TakeFrames(uint8_t* dst, uint32_t limit)
  {
    return ring_.Take(dst, limit);
  }

 private:
  struct ApCsw
  {
    uint8_t ap;
    uint32_t base_csw;
    bool valid;
  };

  bool MeasureFrame(const SubItem* items, uint16_t count, uint32_t& us);
  const ApCsw* FindCsw(uint8_t ap) const;
  void SampleTick(const uint16_t* order, uint16_t due, uint8_t* values, uint8_t* marks);

  MemAp& mem_;

  bool active_ = false;
  SubItem items_[kMaxItems];
  uint16_t count_ = 0U;
  uint32_t period_us_ = 0U;
  bool stop_on_full_ = false;
  bool configured_ = false;
  uint32_t access_us_ = 0U;
  ApCsw csw_[kMaxAps];
  uint8_t csw_count_ = 0U;
  uint64_t start_us_ = 0U;
  uint64_t tick_ = 0U;  ///< 下一拍的序号 / Index of the next tick.

  FrameRing ring_;
};

}  // namespace XrDap::Vendor
