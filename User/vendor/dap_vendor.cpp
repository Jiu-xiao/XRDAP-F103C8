// SPDX-License-Identifier: Apache-2.0
#include "dap_vendor.hpp"

#include <cstring>

#include "timebase.hpp"

namespace XrDap
{
using namespace Vendor;

namespace
{
// 标准 DAP 命令里需要跟踪的几个 / Standard DAP commands this class tracks.
constexpr uint8_t kDapInfo = 0x00U;
constexpr uint8_t kDapHostStatus = 0x01U;
constexpr uint8_t kDapConnect = 0x02U;
constexpr uint8_t kDapDisconnect = 0x03U;
// 命令不支持时的回复 / Reply to an unsupported command.
constexpr uint8_t kDapInvalid = 0xFFU;

// 回复的最小长度：INFO 26 字节，SUB_SET 8 字节，SUB_READ 头 24 字节 / Minimum reply
// sizes: INFO 26 bytes, SUB_SET 8 bytes, SUB_READ header 24 bytes.
constexpr uint16_t kInfoReplyLen = 26U;
constexpr uint16_t kSubSetReplyLen = 8U;
constexpr uint16_t kSubReadHeaderLen = 24U;

enum class SubCtrlOp : uint8_t
{
  STOP = 0,
  START = 1,
  CLEAR = 2,
};

uint64_t NowUs() { return static_cast<uint64_t>(LibXR::Timebase::GetMicroseconds()); }

uint8_t ToU8(Cmd c) { return static_cast<uint8_t>(c); }

bool IsVendor(uint8_t cmd) { return cmd >= kFirstCmd && cmd <= kLastCmd; }
}  // namespace

bool DapVendor::Hook(void* ctx, const uint8_t* req, uint16_t len, uint8_t* resp,
                     uint16_t cap, uint16_t& out)
{
  return static_cast<DapVendor*>(ctx)->OnCommand(req, len, resp, cap, out);
}

void DapVendor::Poll(bool usb_up)
{
  if (usb_up_ && !usb_up)
  {
    Disconnect();
  }
  usb_up_ = usb_up;
  sampler_.Poll();
}

// 目标断开或 USB 掉线：停止采样，扩展命令要求重新 DAP_Connect。
void DapVendor::Disconnect()
{
  sampler_.Stop();
  connected_ = false;
  mem_.Release();
}

bool DapVendor::OnCommand(const uint8_t* req, uint16_t len, uint8_t* resp, uint16_t cap,
                          uint16_t& out)
{
  if (req == nullptr || resp == nullptr || len < 1U || cap < 1U)
  {
    return false;
  }
  const uint8_t cmd = req[0];

  if (sampler_.Active())
  {
    // 采样期间探针被订阅独占：只放行 DAP_Info、DAP_HostStatus 和厂商命令。
    if (cmd == kDapInfo || cmd == kDapHostStatus)
    {
      return false;
    }
    if (!IsVendor(cmd))
    {
      resp[0] = kDapInvalid;
      out = 1U;
      return true;
    }
    return HandleVendor(cmd, req, len, resp, cap, out);
  }

  if (IsVendor(cmd))
  {
    return HandleVendor(cmd, req, len, resp, cap, out);
  }

  // 跟踪 DAP_Connect / DAP_Disconnect，扩展命令要求先连接到 SWD 目标。
  if (cmd == kDapConnect)
  {
    const uint8_t port = (len >= 2U) ? req[1] : 0U;
    connected_ = (port == 0U || port == 1U);
  }
  else if (cmd == kDapDisconnect)
  {
    Disconnect();
  }
  return false;
}

bool DapVendor::HandleVendor(uint8_t cmd, const uint8_t* req, uint16_t len, uint8_t* resp,
                             uint16_t cap, uint16_t& out)
{
  switch (static_cast<Cmd>(cmd))
  {
    case Cmd::INFO:
      return CmdInfo(len, resp, cap, out);
    case Cmd::READ:
      return CmdRead(req, len, resp, cap, out);
    case Cmd::WRITE:
      return CmdWrite(req, len, resp, cap, out);
    case Cmd::SUB_SET:
      return CmdSubSet(req, len, resp, cap, out);
    case Cmd::SUB_CTRL:
      return CmdSubCtrl(req, len, resp, out);
    case Cmd::SUB_READ:
      return CmdSubRead(req, len, resp, cap, out);
  }
  return false;
}

bool DapVendor::CmdInfo(uint16_t len, uint8_t* resp, uint16_t cap, uint16_t& out)
{
  if (cap < kInfoReplyLen)
  {
    out = 0U;
    return true;
  }
  resp[0] = ToU8(Cmd::INFO);
  if (len != 1U)
  {
    return Reply(resp, kStatusArg, out);
  }
  resp[1] = kStatusOk;
  resp[2] = kVersion;
  Put32(&resp[3], kFeatures);
  Put16(&resp[7], kMaxItems);
  resp[9] = kMaxDepth;
  Put32(&resp[10], kRingBytes);
  Put64(&resp[14], NowUs());
  Put32(&resp[22], sampler_.AccessUs());
  out = kInfoReplyLen;
  return true;
}

bool DapVendor::CmdRead(const uint8_t* req, uint16_t len, uint8_t* resp, uint16_t cap,
                        uint16_t& out)
{
  resp[0] = ToU8(Cmd::READ);
  if (len < 2U || req[1] == 0U)
  {
    return Reply(resp, kStatusArg, out);
  }
  const uint8_t count = req[1];
  if (count > kMaxItems)
  {
    return Reply(resp, kStatusResource, out);
  }
  Item items[kMaxItems];
  const uint8_t* p = req + 2;
  const uint8_t* end = req + len;
  uint32_t resp_size = 3U;
  for (uint8_t i = 0; i < count; ++i)
  {
    if (!ParseItem(p, end, items[i], nullptr) || !SizeValid(items[i].size))
    {
      return Reply(resp, kStatusArg, out);
    }
    resp_size += 3U + items[i].size;
  }
  if (p != end)
  {
    return Reply(resp, kStatusArg, out);
  }
  if (resp_size > cap || resp_size > max_packet_)
  {
    return Reply(resp, kStatusResource, out);
  }
  if (!connected_)
  {
    return Reply(resp, kStatusNoTarget, out);
  }
  mem_.ResetCache();
  resp[1] = kStatusOk;
  resp[2] = count;
  uint8_t* q = resp + 3;
  for (uint8_t i = 0; i < count; ++i)
  {
    uint8_t value[8] = {};
    const uint8_t st = mem_.Read(items[i], value);
    Put16(q, items[i].id);
    q[2] = st;
    std::memcpy(q + 3, value, items[i].size);
    q += 3 + items[i].size;
  }
  mem_.Release();
  out = static_cast<uint16_t>(q - resp);
  return true;
}

bool DapVendor::CmdWrite(const uint8_t* req, uint16_t len, uint8_t* resp, uint16_t cap,
                         uint16_t& out)
{
  resp[0] = ToU8(Cmd::WRITE);
  if (len < 2U || req[1] == 0U)
  {
    return Reply(resp, kStatusArg, out);
  }
  const uint8_t count = req[1];
  if (count > kMaxItems)
  {
    return Reply(resp, kStatusResource, out);
  }
  Item items[kMaxItems];
  uint8_t values[kMaxItems][8] = {};
  const uint8_t* p = req + 2;
  const uint8_t* end = req + len;
  for (uint8_t i = 0; i < count; ++i)
  {
    if (!ParseItem(p, end, items[i], nullptr) || !SizeValid(items[i].size) ||
        static_cast<size_t>(end - p) < items[i].size)
    {
      return Reply(resp, kStatusArg, out);
    }
    std::memcpy(values[i], p, items[i].size);
    p += items[i].size;
  }
  if (p != end)
  {
    return Reply(resp, kStatusArg, out);
  }
  if (3U + 3U * count > cap || 3U + 3U * count > max_packet_)
  {
    return Reply(resp, kStatusResource, out);
  }
  if (!connected_)
  {
    return Reply(resp, kStatusNoTarget, out);
  }
  mem_.ResetCache();
  resp[1] = kStatusOk;
  resp[2] = count;
  uint8_t* q = resp + 3;
  for (uint8_t i = 0; i < count; ++i)
  {
    const uint8_t st = mem_.Write(items[i], values[i]);
    Put16(q, items[i].id);
    q[2] = st;
    q += 3;
  }
  mem_.Release();
  out = static_cast<uint16_t>(q - resp);
  return true;
}

bool DapVendor::CmdSubSet(const uint8_t* req, uint16_t len, uint8_t* resp, uint16_t cap,
                          uint16_t& out)
{
  if (cap < kSubSetReplyLen)
  {
    out = 0U;
    return true;
  }
  resp[0] = ToU8(Cmd::SUB_SET);
  if (sampler_.Active())
  {
    return Reply(resp, kStatusState, out);
  }
  if (len < 8U)
  {
    return Reply(resp, kStatusArg, out);
  }
  const uint32_t period = Get32(&req[1]);
  const uint8_t flags = req[5];
  const uint16_t count = Get16(&req[6]);
  if (period == 0U || (flags & 0xFEU) != 0U || count == 0U)
  {
    return Reply(resp, kStatusArg, out);
  }
  if (count > kMaxItems)
  {
    return Reply(resp, kStatusResource, out);
  }
  const uint8_t* p = req + 8;
  const uint8_t* end = req + len;
  SubItem items[kMaxItems];
  uint8_t aps_seen[kMaxAps];
  uint8_t ap_count = 0U;
  for (uint16_t i = 0; i < count; ++i)
  {
    Item it;
    if (!ParseItem(p, end, it, &items[i]) || !SizeValid(it.size) || items[i].div == 0U ||
        items[i].phase >= items[i].div)
    {
      return Reply(resp, kStatusArg, out);
    }
    bool found = false;
    for (uint8_t j = 0; j < ap_count; ++j)
    {
      found = found || (aps_seen[j] == it.ap);
    }
    if (!found)
    {
      if (ap_count >= kMaxAps)
      {
        return Reply(resp, kStatusResource, out);
      }
      aps_seen[ap_count++] = it.ap;
    }
  }
  if (p != end)
  {
    return Reply(resp, kStatusArg, out);
  }
  if (!connected_)
  {
    return Reply(resp, kStatusNoTarget, out);
  }

  Sampler::Estimate est;
  const uint8_t st = sampler_.Configure(items, count, period, (flags & 1U) != 0U, est);
  if (st == kStatusNoTarget)
  {
    return Reply(resp, kStatusNoTarget, out);
  }
  resp[1] = st;
  Put16(&resp[2], est.max_frame);
  Put32(&resp[4], est.frame_us);
  out = kSubSetReplyLen;
  return true;
}

bool DapVendor::CmdSubCtrl(const uint8_t* req, uint16_t len, uint8_t* resp, uint16_t& out)
{
  resp[0] = ToU8(Cmd::SUB_CTRL);
  if (len != 2U)
  {
    return Reply(resp, kStatusArg, out);
  }
  switch (static_cast<SubCtrlOp>(req[1]))
  {
    case SubCtrlOp::STOP:
      sampler_.Stop();
      return Reply(resp, kStatusOk, out);
    case SubCtrlOp::START:
      if (sampler_.Active() || !sampler_.Configured())
      {
        return Reply(resp, kStatusState, out);
      }
      if (!connected_ || !sampler_.Start())
      {
        return Reply(resp, kStatusNoTarget, out);
      }
      return Reply(resp, kStatusOk, out);
    case SubCtrlOp::CLEAR:
      sampler_.ClearFrames();
      return Reply(resp, kStatusOk, out);
  }
  return Reply(resp, kStatusArg, out);
}

bool DapVendor::CmdSubRead(const uint8_t* req, uint16_t len, uint8_t* resp, uint16_t cap,
                           uint16_t& out)
{
  if (cap < kSubReadHeaderLen)
  {
    out = 0U;
    return true;
  }
  resp[0] = ToU8(Cmd::SUB_READ);
  if (len != 3U)
  {
    return Reply(resp, kStatusArg, out);
  }
  if (!sampler_.Configured())
  {
    return Reply(resp, kStatusState, out);
  }
  const uint32_t want = Get16(req + 1);
  const uint32_t room = cap - kSubReadHeaderLen;
  const FrameRing::Taken t =
      sampler_.TakeFrames(resp + kSubReadHeaderLen, (want < room) ? want : room);

  resp[1] = kStatusOk;
  Put64(&resp[2], NowUs());
  Put32(&resp[10], t.remaining);
  Put32(&resp[14], t.dropped);
  Put32(&resp[18], t.skipped);
  Put16(&resp[22], t.frames);
  out = static_cast<uint16_t>(kSubReadHeaderLen + t.bytes);
  return true;
}

}  // namespace XrDap
