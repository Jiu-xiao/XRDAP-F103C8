// SPDX-License-Identifier: Apache-2.0
#pragma once

// SWD 软件时序循环用的寄存器级 GPIO。端口和引脚是模板参数，每次访问编译成对
// BSRR/BRR/IDR 的一次读写；LibXR 的 STM32GPIO 经 HAL 函数调用写引脚，在 -Os 下慢得多。
// 提供 LibXR::Debug::SwdGeneralGPIO 需要的 GpioType 接口（Direction、Pull、SetConfig、
// Write、Read）。端口时钟由 CubeMX 的 MX_GPIO_Init 打开。
//
// Write() 和 Read() 经 BSRR/BRR/IDR，是原子的。SetConfig() 对 CRL/CRH 做读-改-写，同一个
// 寄存器还管着这个端口的另外 7 个引脚，所以只能在一个执行上下文里用（本固件是主循环里的
// SWD），不能和中断里改同一端口模式的代码并存。
//
// Register-level GPIO for the SWD bit-bang loop. Port and pin are template arguments, so
// every access compiles to one store or load on BSRR/BRR/IDR; LibXR's STM32GPIO writes a
// pin through HAL function calls, which is much slower under -Os. It provides the
// GpioType interface LibXR::Debug::SwdGeneralGPIO expects (Direction, Pull, SetConfig,
// Write, Read). The port clock is enabled by CubeMX's MX_GPIO_Init.
//
// Write() and Read() go through BSRR/BRR/IDR and are atomic. SetConfig() does a
// read-modify-write of CRL/CRH, which also hold 7 other pins of the port, so it is for
// one execution context only (here: SWD in the main loop) and must not coexist with
// interrupt code that changes pin modes on the same port.

#include <cstdint>

#include "libxr_def.hpp"
#include "main.h"

namespace XrDap
{

template <uintptr_t PORT, uint8_t PIN>
class FastGpio
{
  static_assert(PIN < 16, "STM32F1 GPIO ports have 16 pins");

 public:
  enum class Direction : uint8_t
  {
    INPUT,
    OUTPUT_PUSH_PULL,
    OUTPUT_OPEN_DRAIN
  };
  enum class Pull : uint8_t
  {
    NONE,
    UP,
    DOWN
  };
  struct Configuration
  {
    Direction direction;
    Pull pull;
  };

  LibXR::ErrorCode SetConfig(Configuration config)
  {
    uint32_t mode = kInputFloating;
    switch (config.direction)
    {
      case Direction::OUTPUT_PUSH_PULL:
        mode = kOutputPushPull50MHz;
        break;
      case Direction::OUTPUT_OPEN_DRAIN:
        mode = kOutputOpenDrain50MHz;
        break;
      case Direction::INPUT:
        if (config.pull != Pull::NONE)
        {
          mode = kInputPull;
          // In pull mode the output latch selects up (1) or down (0).
          Write(config.pull == Pull::UP);
        }
        break;
    }
    volatile uint32_t& cr = (PIN < 8) ? Regs()->CRL : Regs()->CRH;
    cr = (cr & ~(0xFU << kShift)) | (mode << kShift);
    return LibXR::ErrorCode::OK;
  }

  inline void Write(bool high)
  {
    if (high)
    {
      Regs()->BSRR = kMask;
    }
    else
    {
      Regs()->BRR = kMask;
    }
  }

  inline bool Read() const { return (Regs()->IDR & kMask) != 0U; }

 private:
  static constexpr uint32_t kInputFloating = 0x4U;
  static constexpr uint32_t kInputPull = 0x8U;
  static constexpr uint32_t kOutputPushPull50MHz = 0x3U;
  static constexpr uint32_t kOutputOpenDrain50MHz = 0x7U;
  static constexpr uint32_t kShift = (PIN % 8U) * 4U;
  static constexpr uint32_t kMask = 1U << PIN;

  static GPIO_TypeDef* Regs() { return reinterpret_cast<GPIO_TypeDef*>(PORT); }
};

}  // namespace XrDap
