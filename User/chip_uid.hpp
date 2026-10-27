// SPDX-License-Identifier: Apache-2.0
#pragma once

// The 96-bit unique id of the STM32F1 (reference manual, "Device electronic signature").
// It is the id part of the probe's serial.
//
// STM32F1 的 96 位芯片唯一标识（参考手册“Device electronic signature”），是探针序列号
// 的标识部分。

#include <cstdint>

namespace XrDap
{

/// 唯一标识的字节数 / Bytes of the unique id.
inline constexpr uint32_t kChipUidBytes = 12U;

/// 唯一标识（只读，可在任何上下文读）/ The unique id (read-only, readable from any
/// context).
inline const uint8_t* ChipUid() { return reinterpret_cast<const uint8_t*>(0x1FFFF7E8U); }

}  // namespace XrDap
