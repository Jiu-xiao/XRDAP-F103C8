// SPDX-License-Identifier: Apache-2.0
#include "mem_ap.hpp"

#include <cstring>

#include "timebase.hpp"

namespace XrDap::Vendor
{
namespace
{
using Ack = LibXR::Debug::SwdProtocol::Ack;
using DpReadReg = LibXR::Debug::SwdProtocol::DpReadReg;

// MEM-AP 寄存器（A[3:2]）/ MEM-AP registers (A[3:2]).
constexpr uint8_t kApCsw = 0x00U;
constexpr uint8_t kApTar = 0x01U;
constexpr uint8_t kApDrw = 0x03U;
// CSW 的 Size（位 2:0）和 AddrInc（位 5:4）/ CSW Size (bits 2:0) and AddrInc (bits 5:4).
constexpr uint32_t kCswSizeAndInc = 0x37U;
// CTRL/STAT 的 STICKYORUN、STICKYERR、WDATAERR / CTRL/STAT STICKYORUN, STICKYERR,
// WDATAERR.
constexpr uint32_t kCtrlStatErrors = (1U << 1) | (1U << 5) | (1U << 7);
// ABORT 的 STKCMPCLR、STKERRCLR、WDERRCLR、ORUNERRCLR / ABORT STKCMPCLR, STKERRCLR,
// WDERRCLR, ORUNERRCLR.
constexpr uint32_t kAbortClearAll = 0x1EU;

bool TxnOk(LibXR::ErrorCode ec, Ack ack)
{
  return ec == LibXR::ErrorCode::OK && ack == Ack::OK;
}

uint8_t SizeCode(uint8_t size) { return (size == 1U) ? 0U : (size == 2U) ? 1U : 2U; }
}  // namespace

void MemAp::ResetCache()
{
  for (ApState& st : ap_)
  {
    st = ApState{};
  }
  ap_count_ = 0U;
}

MemAp::ApState* MemAp::EnsureAp(uint8_t ap)
{
  for (size_t i = 0; i < ap_count_; ++i)
  {
    if (ap_[i].ap == ap)
    {
      return ap_[i].valid ? &ap_[i] : nullptr;
    }
  }
  // 表满时复用最后一格，代价是每次重新读 CSW。
  ApState* st = (ap_count_ >= kMaxAps) ? &ap_[kMaxAps - 1U] : &ap_[ap_count_++];
  st->ap = ap;
  st->valid = ReadCsw(ap, st->base_csw);
  st->written = false;
  return st->valid ? st : nullptr;
}

// 刚重新连接，或上一次访问被打断（例如采样被停掉）之后，第一次 AP 访问偶尔会因 DP 里
// 残留的粘滞错误失败，所以失败时清掉再试一次。
bool MemAp::ReadCsw(uint8_t ap, uint32_t& csw)
{
  for (int attempt = 0; attempt < 2; ++attempt)
  {
    Ack ack;
    if (TxnOk(swd_.SetSelectCached(static_cast<uint32_t>(ap) << 24, ack), ack) &&
        TxnOk(swd_.ApReadTxn(kApCsw, csw, ack), ack))
    {
      return true;
    }
    ClearSticky();
    swd_.InvalidateSelectCache();
  }
  return false;
}

bool MemAp::Calibrate(uint8_t ap, uint32_t& us)
{
  constexpr uint32_t kRounds = 16U;
  Ack ack;
  uint32_t v = 0U;
  bool ok = TxnOk(swd_.SetSelectCached(static_cast<uint32_t>(ap) << 24, ack), ack);
  const uint64_t t0 = LibXR::Timebase::GetMicroseconds();
  for (uint32_t i = 0; ok && i < kRounds; ++i)
  {
    ok = TxnOk(swd_.ApWriteTxn(kApTar, 0U, ack), ack) &&
         TxnOk(swd_.ApReadTxn(kApCsw, v, ack), ack);
  }
  const uint64_t t1 = LibXR::Timebase::GetMicroseconds();
  if (!ok)
  {
    ClearSticky();
    return false;
  }
  const uint32_t per = static_cast<uint32_t>((t1 - t0 + kRounds - 1U) / kRounds);
  us = (per == 0U) ? 1U : per;
  return true;
}

// 写 CSW：保留原值里 Size 和 AddrInc 以外的位，关闭自增。
bool MemAp::SetSize(ApState& st, uint8_t size)
{
  if (!st.valid)
  {
    return false;
  }
  const uint32_t csw = (st.base_csw & ~kCswSizeAndInc) | SizeCode(size);
  if (st.written && st.written_csw == csw)
  {
    return true;
  }
  Ack ack;
  if (!TxnOk(swd_.ApWriteTxn(kApCsw, csw, ack), ack))
  {
    st.written = false;
    ClearSticky();
    return false;
  }
  st.written = true;
  st.written_csw = csw;
  return true;
}

bool MemAp::ReadWord(uint32_t addr, uint32_t& val)
{
  Ack ack;
  if (!TxnOk(swd_.ApWriteTxn(kApTar, addr, ack), ack) ||
      !TxnOk(swd_.ApReadTxn(kApDrw, val, ack), ack))
  {
    ClearSticky();
    return false;
  }
  return true;
}

// 写是 posted 的，写完读一次 CTRL/STAT 看有没有粘滞错误。
bool MemAp::CheckCtrlStat()
{
  Ack ack;
  uint32_t cs = 0U;
  if (!TxnOk(swd_.DpReadTxn(DpReadReg::CTRL_STAT, cs, ack), ack) ||
      (cs & kCtrlStatErrors) != 0U)
  {
    ClearSticky();
    return false;
  }
  return true;
}

bool MemAp::WriteWord(uint32_t addr, uint32_t val)
{
  Ack ack;
  if (!TxnOk(swd_.ApWriteTxn(kApTar, addr, ack), ack) ||
      !TxnOk(swd_.ApWriteTxn(kApDrw, val, ack), ack))
  {
    ClearSticky();
    return false;
  }
  return CheckCtrlStat();
}

// 不清掉粘滞错误，后面的访问会连续失败。
void MemAp::ClearSticky()
{
  Ack ack;
  (void)swd_.WriteAbortTxn(kAbortClearAll, ack);
}

bool MemAp::Resolve(const Item& it, uint32_t& addr, uint8_t& st)
{
  uint32_t cur = it.base;
  for (uint8_t i = 0; i < it.n; ++i)
  {
    const uint32_t a = cur + static_cast<uint32_t>(static_cast<int32_t>(it.deref[i]));
    if ((a & 3U) != 0U)
    {
      st = static_cast<uint8_t>(kItemAlignChain + i);
      return false;
    }
    uint32_t p = 0U;
    if (!ReadWord(a, p))
    {
      st = static_cast<uint8_t>(kItemSwdChain + i);
      return false;
    }
    if (p == 0U)
    {
      st = static_cast<uint8_t>(kItemNull + i);
      return false;
    }
    cur = p;
  }
  const uint32_t a = cur + static_cast<uint32_t>(static_cast<int32_t>(it.final_off));
  const uint8_t align = (it.size == 8U) ? 4U : it.size;
  if ((a & (align - 1U)) != 0U)
  {
    st = kItemAlignLast;
    return false;
  }
  addr = a;
  st = kItemOk;
  return true;
}

// 1、2 字节在 DRW 的对应字节通道里；8 字节是两次 32 位读，不是原子的。
uint8_t MemAp::ReadValue(ApState& ap, uint8_t size, uint32_t addr, uint8_t* out)
{
  if (!SetSize(ap, (size == 8U) ? 4U : size))
  {
    return kItemSwdLast;
  }
  uint32_t v = 0U;
  if (!ReadWord(addr, v))
  {
    return kItemSwdLast;
  }
  if (size == 8U)
  {
    uint32_t v2 = 0U;
    if (!ReadWord(addr + 4U, v2))
    {
      return kItemSwdLast;
    }
    std::memcpy(out, &v, 4);
    std::memcpy(out + 4, &v2, 4);
    return kItemOk;
  }
  if (size == 4U)
  {
    std::memcpy(out, &v, 4);
    return kItemOk;
  }
  const uint32_t shifted = v >> ((addr & 3U) * 8U);
  if (size == 1U)
  {
    out[0] = static_cast<uint8_t>(shifted);
  }
  else
  {
    const uint16_t h = static_cast<uint16_t>(shifted);
    std::memcpy(out, &h, 2);
  }
  return kItemOk;
}

// 解引用前 CSW 必须是 32 位访问：上一个条目可能把它改成了 1 或 2 字节。
uint8_t MemAp::ReadItem(ApState& ap, const Item& it, uint8_t* out)
{
  if (it.n != 0U && !SetSize(ap, 4U))
  {
    return kItemSwdLast;
  }
  uint32_t addr = 0U;
  uint8_t st = kItemOk;
  if (!Resolve(it, addr, st))
  {
    return st;
  }
  return ReadValue(ap, it.size, addr, out);
}

uint8_t MemAp::Read(const Item& it, uint8_t* out)
{
  ApState* ap = EnsureAp(it.ap);
  return (ap == nullptr) ? kItemSwdLast : ReadItem(*ap, it, out);
}

uint8_t MemAp::Write(const Item& it, const uint8_t* value)
{
  ApState* ap = EnsureAp(it.ap);
  if (ap == nullptr || (it.n != 0U && !SetSize(*ap, 4U)))
  {
    return kItemSwdLast;
  }
  uint32_t addr = 0U;
  uint8_t st = kItemOk;
  if (!Resolve(it, addr, st))
  {
    return st;
  }
  if (it.size == 8U)
  {
    uint32_t lo = 0U;
    uint32_t hi = 0U;
    std::memcpy(&lo, value, 4);
    std::memcpy(&hi, value + 4, 4);
    if (!SetSize(*ap, 4U) || !WriteWord(addr, lo) || !WriteWord(addr + 4U, hi))
    {
      return kItemSwdLast;
    }
    return kItemOk;
  }
  if (!SetSize(*ap, it.size))
  {
    return kItemSwdLast;
  }
  uint32_t word = 0U;
  const uint32_t lane = (addr & 3U) * 8U;
  if (it.size == 4U)
  {
    std::memcpy(&word, value, 4);
  }
  else if (it.size == 2U)
  {
    uint16_t h = 0U;
    std::memcpy(&h, value, 2);
    word = static_cast<uint32_t>(h) << lane;
  }
  else
  {
    word = static_cast<uint32_t>(value[0]) << lane;
  }
  return WriteWord(addr, word) ? kItemOk : kItemSwdLast;
}

}  // namespace XrDap::Vendor
