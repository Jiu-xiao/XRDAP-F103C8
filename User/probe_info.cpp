// SPDX-License-Identifier: Apache-2.0
#include "probe_info.hpp"

namespace XrDap
{

// 不用 snprintf：只为这一处会把 printf 整套拉进 64 KB 镜像。
// No snprintf: it would pull the whole printf family into the 64 KB image for this alone.
const char* ProbeSerial()
{
  static char serial[sizeof(kSerialPrefix) + 2 * kChipUidSize] = {};
  if (serial[0] == '\0')
  {
    constexpr char kHex[] = "0123456789ABCDEF";
    char* p = serial;
    for (const char* s = kSerialPrefix; *s != '\0'; ++s)
    {
      *p++ = *s;
    }
    for (uint32_t i = 0; i < kChipUidSize; ++i)
    {
      *p++ = kHex[ChipUid()[i] >> 4];
      *p++ = kHex[ChipUid()[i] & 0x0F];
    }
  }
  return serial;
}

}  // namespace XrDap
