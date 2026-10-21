// SPDX-License-Identifier: Apache-2.0
#pragma once

// 经 SWD 访问 MEM-AP：沿指针链读写目标内存。自己管理 SELECT、CSW、TAR，不走 DAP 的传输
// 流水线。DAP 命令和采样都在主循环里执行，不会同时驱动 SWD，所以没有锁；但每次扩展访问
// 结束都要调用 Release()，让 SWD 的 SELECT 缓存失效：上位机须把 SELECT、CSW、TAR 视为未知。
//
// MEM-AP access over SWD: reads and writes target memory along pointer chains. It manages
// SELECT, CSW and TAR itself, bypassing the DAP transfer pipeline. DAP commands and
// sampling both run in the main loop and never drive SWD at the same time, so there is
// no lock; every extension access still ends with Release(), which invalidates the SWD
// SELECT cache: the host must treat SELECT, CSW and TAR as unknown.

#include <cstdint>

#include "debug/swd.hpp"
#include "vendor_protocol.hpp"

namespace XrDap::Vendor
{

/// SWD 链路上的 MEM-AP 访问 / MEM-AP access on the SWD link.
class MemAp
{
 public:
  /// 一个 AP 的 CSW 状态 / CSW state of one AP.
  struct ApState
  {
    /// AP 号；0xFF 表示还没有选 AP / AP index; 0xFF before any AP is chosen.
    uint8_t ap = 0xFF;
    /// 读出的 CSW 原值，保留 Size、AddrInc 以外的位 / CSW as read; bits other than Size
    /// and AddrInc are preserved.
    uint32_t base_csw = 0;
    /// base_csw 读成功 / base_csw was read successfully.
    bool valid = false;
    /// 最近一次写入的 CSW，用来省掉重复写 / Last CSW written, to skip repeated writes.
    uint32_t written_csw = 0;
    /// written_csw 有效 / written_csw is valid.
    bool written = false;
  };

  /// @param swd SWD 实现，生命周期需长于本对象 / SWD implementation; must outlive this
  ///        object.
  explicit MemAp(LibXR::Debug::Swd& swd) : swd_(swd) {}

  /// @brief 一次扩展访问结束：让 SWD 的 SELECT 缓存失效 / End of one extension access:
  ///        invalidate the SWD SELECT cache.
  void Release() { swd_.InvalidateSelectCache(); }

  /// 清空命令用的 AP 缓存，每条命令开始时调用 / Clear the per-command AP cache; call at
  /// the start of every command.
  void ResetCache();
  /// @brief 本命令里第一次用到某个 AP 时读一次它的 CSW / Read an AP's CSW the first time
  ///        a command uses it.
  /// @return 读不到 CSW 时返回 nullptr；缓存满时复用最后一格 / nullptr when the CSW
  ///         cannot be read; the last slot is reused when the cache is full.
  ApState* EnsureAp(uint8_t ap);
  /// 读一个 AP 的 CSW，失败时清粘滞错误再试一次 / Read an AP's CSW; on failure clear the
  /// sticky errors and retry once.
  bool ReadCsw(uint8_t ap, uint32_t& csw);
  /// 读一个条目，返回条目状态 / Read one item; returns the item status.
  uint8_t Read(const Item& it, uint8_t* out);
  /// 写一个条目，返回条目状态 / Write one item; returns the item status.
  uint8_t Write(const Item& it, const uint8_t* value);
  /// 用给定的 AP 状态读一个条目（采样用），返回条目状态 / Read one item with a given AP
  /// state (used by sampling); returns the item status.
  uint8_t ReadItem(ApState& ap, const Item& it, uint8_t* out);
  /// @brief 标定一次访问（写 TAR + 读 AP 寄存器）的耗时；读的是 CSW，没有副作用 /
  ///        Calibrate one access (TAR write + AP register read); reads CSW, no side
  ///        effects.
  /// @param us 成功时为单次访问耗时（微秒，至少 1）/ On success the per-access time in
  ///        microseconds, at least 1.
  bool Calibrate(uint8_t ap, uint32_t& us);

 private:
  bool SetSize(ApState& st, uint8_t size);
  bool ReadWord(uint32_t addr, uint32_t& val);
  bool WriteWord(uint32_t addr, uint32_t val);
  bool CheckCtrlStat();
  void ClearSticky();
  bool Resolve(const Item& it, uint32_t& addr, uint8_t& st);
  uint8_t ReadValue(ApState& ap, uint8_t size, uint32_t addr, uint8_t* out);

  LibXR::Debug::Swd& swd_;
  ApState ap_[kMaxAps];
  uint8_t ap_count_ = 0U;
};

}  // namespace XrDap::Vendor
