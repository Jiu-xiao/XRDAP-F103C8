// SPDX-License-Identifier: Apache-2.0
#pragma once

// XRDAP-Core 的平台部分：采样帧缓冲的大小。
//
// Platform part of XRDAP-Core: the size of the sample frame buffer.

#include <cstdint>

namespace XrDap::Port
{
/// 采样帧缓冲的字节数，2 的幂 / Bytes of the sample frame buffer, a power of two.
inline constexpr uint32_t kRingBytes = 4096U;

}  // namespace XrDap::Port
