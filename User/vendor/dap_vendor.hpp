// SPDX-License-Identifier: Apache-2.0
#pragma once

// XR 厂商扩展命令（0x80 到 0x85）：多地址读写、指针链解引用、探针端周期采样。通过 libxr
// DAP 类的命令钩子接入，协议与 XRDAP-ESP 相同（见该仓库 docs/protocol.md 第 3 节）。
//
// 命令和采样都在主循环里执行：钩子在 dap.Step() 里被调用，采样由主循环每圈调用 Poll()。
// USB 是唯一的传输；USB 掉线（总线复位或拔出）时采样停止。
//
// XR vendor extension commands (0x80 to 0x85): multi-address read/write, pointer-chain
// dereference and probe-side periodic sampling. They plug in through the libxr DAP class
// command hook; the protocol matches XRDAP-ESP (see section 3 of its docs/protocol.md).
//
// Commands and sampling both run in the main loop: the hook is called from dap.Step(),
// and the main loop calls Poll() on every pass for sampling. USB is the only transport;
// sampling stops when USB goes away (bus reset or unplug).

#include <cstdint>

#include "dap_activity.hpp"
#include "debug/swd.hpp"
#include "mem_ap.hpp"
#include "sampler.hpp"
#include "vendor_protocol.hpp"

namespace XrDap
{

/// XR 厂商扩展命令处理器 / XR vendor extension command handler.
class DapVendor
{
 public:
  /// @param swd SWD 实现，生命周期需长于本对象 / SWD implementation; must outlive this
  ///        object.
  /// @param max_packet DAP 包长 / DAP packet size.
  DapVendor(LibXR::Debug::Swd& swd, uint16_t max_packet)
      : max_packet_(max_packet), mem_(swd), sampler_(mem_)
  {
  }

  /// @brief 主循环每圈调用：USB 掉线时停止采样，否则读到期的一拍 / Called on every
  ///        main-loop pass: stops sampling when USB went away, otherwise reads a due
  ///        tick.
  /// @param usb_up DAP 接口当前已被主机配置 / The DAP interface is currently configured
  ///        by the host.
  void Poll(bool usb_up);

  /// 是否正在采样（主循环据此不在 Step 里等待）/ Whether sampling runs (the main loop
  /// then does not wait in Step).
  bool Sampling() const { return sampler_.Active(); }

  /// DAP 活动和上位机状态，给状态灯读 / DAP activity and host state, read by the status
  /// light.
  const Vendor::DapActivity& Activity() const { return activity_; }

  /// libxr DAP 类的命令钩子入口，返回 true 表示命令已处理（回复在 resp/out）/ Entry for
  /// the libxr DAP class command hook; returns true when the command was handled (reply
  /// in resp/out).
  static bool Hook(void* ctx, const uint8_t* req, uint16_t len, uint8_t* resp,
                   uint16_t cap, uint16_t& out);

 private:
  bool OnCommand(const uint8_t* req, uint16_t len, uint8_t* resp, uint16_t cap,
                 uint16_t& out);
  void Disconnect();
  bool HandleVendor(uint8_t cmd, const uint8_t* req, uint16_t len, uint8_t* resp,
                    uint16_t cap, uint16_t& out);
  bool CmdInfo(uint16_t len, uint8_t* resp, uint16_t cap, uint16_t& out);
  bool CmdRead(const uint8_t* req, uint16_t len, uint8_t* resp, uint16_t cap,
               uint16_t& out);
  bool CmdWrite(const uint8_t* req, uint16_t len, uint8_t* resp, uint16_t cap,
                uint16_t& out);
  bool CmdSubSet(const uint8_t* req, uint16_t len, uint8_t* resp, uint16_t cap,
                 uint16_t& out);
  bool CmdSubCtrl(const uint8_t* req, uint16_t len, uint8_t* resp, uint16_t& out);
  bool CmdSubRead(const uint8_t* req, uint16_t len, uint8_t* resp, uint16_t cap,
                  uint16_t& out);

  uint16_t max_packet_;
  Vendor::MemAp mem_;
  Vendor::Sampler sampler_;
  Vendor::DapActivity activity_;
  bool connected_ = false;
  bool usb_up_ = false;
};

}  // namespace XrDap
