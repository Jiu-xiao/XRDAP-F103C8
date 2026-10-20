// SPDX-License-Identifier: Apache-2.0
#pragma once

// XR 厂商扩展的线上格式：命令号、状态码、上限、条目和小端字节序工具。协议见
// XRDAP-ESP 仓库 docs/protocol.md 第 3 节。
//
// Wire format of the XR vendor extension: command bytes, status codes, limits, items and
// little-endian byte helpers. See section 3 of docs/protocol.md in XRDAP-ESP.

#include <cstddef>
#include <cstdint>

namespace XrDap::Vendor
{

/// 厂商命令号 / Vendor command bytes.
enum class Cmd : uint8_t
{
  INFO = 0x80,      ///< 能力和探针时间 / Capabilities and probe time.
  READ = 0x81,      ///< 多条目读 / Multi-item read.
  WRITE = 0x82,     ///< 多条目写 / Multi-item write.
  SUB_SET = 0x83,   ///< 配置采样 / Configure sampling.
  SUB_CTRL = 0x84,  ///< 开始、停止、清空采样 / Start, stop, clear sampling.
  SUB_READ = 0x85,  ///< 取采样帧 / Read sample frames.
};

/// 内置命令的范围 / Range of the built-in commands.
constexpr uint8_t kFirstCmd = static_cast<uint8_t>(Cmd::INFO);
constexpr uint8_t kLastCmd = static_cast<uint8_t>(Cmd::SUB_READ);

/// 协议版本 / Protocol version.
constexpr uint8_t kVersion = 1;
/// 功能位：READ | WRITE | 采样 | 8 字节访问 / Feature bits: READ | WRITE | sampling |
/// 8-byte access.
constexpr uint32_t kFeatures = 0x0F;
/// 一条命令最多的条目数 / Maximum items per command.
constexpr uint16_t kMaxItems = 32;
/// 指针链最大深度 / Maximum pointer-chain depth.
constexpr uint8_t kMaxDepth = 8;
/// 一条命令里最多用到的 AP 数 / Maximum distinct APs in one command.
constexpr size_t kMaxAps = 8;
/// 采样环形缓冲字节数，必须是 2 的幂。F103 只有 20 KB RAM，取 4 KB / Sampling ring
/// buffer size in bytes; must be a power of two. The F103 has 20 KB of RAM, so 4 KB.
constexpr uint32_t kRingBytes = 4096;
/// 一帧采样数据的最大字节数：节拍号 4 + 时间戳 6 + 状态位图 + 值 / Maximum bytes of one
/// sample frame: tick 4 + timestamp 6 + status bitmap + values.
constexpr uint16_t kMaxFrameBytes = 10U + (kMaxItems + 7U) / 8U + kMaxItems * 8U;

/// 命令状态：成功 / Command status: success.
constexpr uint8_t kStatusOk = 0x00;
/// 命令状态：参数错误 / Command status: bad argument.
constexpr uint8_t kStatusArg = 0x01;
/// 命令状态：当前状态不允许 / Command status: not allowed in the current state.
constexpr uint8_t kStatusState = 0x02;
/// 命令状态：资源不足 / Command status: insufficient resources.
constexpr uint8_t kStatusResource = 0x03;
/// 命令状态：未连接目标 / Command status: no target connected.
constexpr uint8_t kStatusNoTarget = 0x04;

/// 条目状态：成功 / Item status: success.
constexpr uint8_t kItemOk = 0x00;
/// 条目状态 0x10 + k：第 k 级读指针时 SWD 出错或超时 / Item status 0x10 + k: SWD
/// error or timeout reading the pointer at level k.
constexpr uint8_t kItemSwdChain = 0x10;
/// 条目状态：最后一次读或写时 SWD 失败 / Item status: SWD failed on the final access.
constexpr uint8_t kItemSwdLast = 0x1F;
/// 条目状态 0x20 + k：第 k 级的读地址未 4 字节对齐 / Item status 0x20 + k: the read
/// address at level k is not 4-byte aligned.
constexpr uint8_t kItemAlignChain = 0x20;
/// 条目状态：最后访问的地址未按 size 对齐 / Item status: the final address is not
/// aligned to size.
constexpr uint8_t kItemAlignLast = 0x2F;
/// 条目状态 0x30 + k：第 k 级读出的指针为 0，链终止 / Item status 0x30 + k: the pointer
/// read at level k is 0 and the chain stops.
constexpr uint8_t kItemNull = 0x30;

/// 一个读写或采样条目：沿指针链解引用后访问 size 字节 / One read, write or sampling
/// item: dereference along a pointer chain, then access size bytes.
struct Item
{
  /// AP 号 / AP index.
  uint8_t ap = 0;
  /// 主机分配的条目号 / Host-assigned item id.
  uint16_t id = 0;
  /// 访问字节数，1、2、4 或 8 / Bytes to access: 1, 2, 4 or 8.
  uint8_t size = 0;
  /// 起始地址 / Start address.
  uint32_t base = 0;
  /// 指针链深度，不超过 kMaxDepth / Pointer-chain depth, at most kMaxDepth.
  uint8_t n = 0;
  /// 每级解引用前加的偏移 / Offset added before each dereference.
  int16_t deref[kMaxDepth] = {};
  /// 最后一次访问前加的偏移 / Offset added before the final access.
  int16_t final_off = 0;
};

/// 一个采样条目：每 div 拍在第 phase 拍读一次 / One sampling item, read on ticks where
/// (tick + phase) % div == 0.
struct SubItem
{
  Item item;
  uint16_t div = 1;
  uint16_t phase = 0;
};

inline void Put16(uint8_t* p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}

/// 写 bytes 个字节的小端整数 / Write a little-endian integer of `bytes` bytes.
inline void PutLe(uint8_t* p, uint64_t v, int bytes)
{
  for (int i = 0; i < bytes; ++i)
  {
    p[i] = static_cast<uint8_t>(v >> (8 * i));
  }
}

inline void Put32(uint8_t* p, uint32_t v) { PutLe(p, v, 4); }
inline void Put48(uint8_t* p, uint64_t v) { PutLe(p, v, 6); }
inline void Put64(uint8_t* p, uint64_t v) { PutLe(p, v, 8); }

inline uint16_t Get16(const uint8_t* p)
{
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

inline uint32_t Get32(const uint8_t* p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

/// @brief 填只有状态的回复（resp[0] 已是命令号）/ Fill a status-only reply (resp[0]
///        already holds the command byte).
/// @return 总是 true，方便 `return Reply(...)` / Always true, for `return Reply(...)`.
inline bool Reply(uint8_t* resp, uint8_t status, uint16_t& out)
{
  resp[1] = status;
  out = 2U;
  return true;
}

/// 访问字节数是否合法 / Whether an access size is valid.
inline bool SizeValid(uint8_t size)
{
  return size == 1U || size == 2U || size == 4U || size == 8U;
}

/// @brief 解析一个条目，成功时 p 前进到条目之后 / Parse one item; on success p moves past
///        it.
/// @param sub 不为空时条目后还有 div 和 phase（采样条目）/ When not null, div and phase
///        follow the item (sampling item).
/// @return 长度不足或深度超限时返回 false / False when truncated or too deep.
inline bool ParseItem(const uint8_t*& p, const uint8_t* end, Item& it, SubItem* sub)
{
  if (static_cast<size_t>(end - p) < 9U)
  {
    return false;
  }
  it.ap = p[0];
  it.id = Get16(&p[1]);
  it.size = p[3];
  it.base = Get32(&p[4]);
  it.n = p[8];
  if (it.n > kMaxDepth)
  {
    return false;
  }
  p += 9;
  if (static_cast<size_t>(end - p) < static_cast<size_t>(it.n) * 2U + 2U)
  {
    return false;
  }
  for (uint8_t i = 0; i < it.n; ++i)
  {
    it.deref[i] = static_cast<int16_t>(Get16(p));
    p += 2;
  }
  it.final_off = static_cast<int16_t>(Get16(p));
  p += 2;
  if (sub != nullptr)
  {
    if (static_cast<size_t>(end - p) < 4U)
    {
      return false;
    }
    sub->item = it;
    sub->div = Get16(p);
    sub->phase = Get16(p + 2);
    p += 4;
  }
  return true;
}

/// @brief 已校验过的一串采样条目，留在请求字节里按需解析 / A validated run of sampling
///        items, left in the request bytes and parsed on demand.
struct SubItemList
{
  const uint8_t* begin;
  const uint8_t* end;
  uint16_t count;

  /// 依次解析每个条目并调用 fn(const SubItem&) / Parse each item in turn and call
  /// fn(const SubItem&).
  template <typename Fn>
  void ForEach(Fn&& fn) const
  {
    const uint8_t* p = begin;
    for (uint16_t i = 0; i < count; ++i)
    {
      Item it;
      SubItem sub;
      (void)ParseItem(p, end, it, &sub);
      fn(sub);
    }
  }
};

}  // namespace XrDap::Vendor
