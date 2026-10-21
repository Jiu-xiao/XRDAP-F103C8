// SPDX-License-Identifier: Apache-2.0
#pragma once

// 本探针的身份：产品名、板子名、固件版本和芯片 UID。序列号规则和 DAP_Info
// 字符串来自 XRDAP-Core 的 probe_identity.hpp：前缀 + 12 字节 UID
// 的大写十六进制，与 USB 协议栈生成 的设备序列号相同。
//
// Identity of this probe: product, board, firmware version and chip UID. The
// serial rule and the DAP_Info strings come from XRDAP-Core's
// probe_identity.hpp: prefix + the 12-byte UID in upper-case hex, the same
// string the USB stack builds for the device serial.

#include <cstdint>

#include "probe_identity.hpp"

namespace XrDap
{

inline constexpr char kFirmwareVersion[] = "0.6.0";
inline constexpr char kProductName[] = "XR-DAP-F103";
inline constexpr char kUsbProductName[] = "XR-DAP-F103 CMSIS-DAP";
inline constexpr char kBoardName[] = "STLINK-V2";

/// 芯片 UID 的地址与长度 / Address and length of the chip UID.
inline const uint8_t* ChipUid() { return reinterpret_cast<const uint8_t*>(0x1FFFF7E8U); }
inline constexpr uint32_t kChipUidSize = 12;

}  // namespace XrDap
