// SPDX-License-Identifier: Apache-2.0
#pragma once

// SWD 位带循环用的寄存器级 GPIO（STM32F1）。端口基址和引脚是模板参数，每次访问编译成一次
// 读写，libxr 的 SwdGeneralGPIO 因此能跑到最高速。提供它要的 GpioType 接口（Direction、
// Pull、Configuration、SetConfig、Write、Read）。端口时钟在构造前打开。
//
// Register-level GPIO for the SWD bit-bang loop on the STM32F1. Port base and pin are
// template arguments, so every access compiles to one load or store and libxr's
// SwdGeneralGPIO reaches its top rate. It provides the GpioType interface that class
// expects (Direction, Pull, Configuration, SetConfig, Write, Read). The port clock is
// enabled before construction.

#include <cstdint>

#include "libxr_def.hpp"

namespace XrDap
{

/// 一个 GPIO 端口的寄存器 / Registers of one GPIO port.
struct F1GpioRegs
{
  volatile uint32_t cfg_low;   ///< CRL / CFGLR
  volatile uint32_t cfg_high;  ///< CRH / CFGHR
  volatile uint32_t input;     ///< IDR / INDR
  volatile uint32_t output;    ///< ODR / OUTDR
  volatile uint32_t set;       ///< BSRR / BSHR
  volatile uint32_t reset;     ///< BRR / BCR
};

/// @brief 一个引脚 / One pin.
/// @tparam PORT 端口寄存器基址 / Port register base address.
/// @tparam PIN 引脚号，0 到 15 / Pin number, 0 to 15.
/// @note 只在驱动 SWD 的上下文里调用（libxr DAP 执行上下文或采样）/ Call only from the
///       context that drives SWD (the libxr DAP execution context or sampling).
template <uintptr_t PORT, uint8_t PIN>
class F1Gpio
{
  static_assert(PIN < 16, "F1-style GPIO ports have 16 pins");

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
          // 上下拉模式下输出锁存决定上拉（1）还是下拉（0）/ In pull mode the output
          // latch selects up (1) or down (0).
          Write(config.pull == Pull::UP);
        }
        break;
    }
    volatile uint32_t& cfg = (PIN < 8) ? Regs()->cfg_low : Regs()->cfg_high;
    cfg = (cfg & ~(0xFU << kShift)) | (mode << kShift);
    return LibXR::ErrorCode::OK;
  }

  [[gnu::always_inline]] inline void Write(bool high)
  {
    if (high)
    {
      Regs()->set = kMask;
    }
    else
    {
      Regs()->reset = kMask;
    }
  }

  [[gnu::always_inline]] inline bool Read() const
  {
    return (Regs()->input & kMask) != 0U;
  }

 private:
  static constexpr uint32_t kInputFloating = 0x4U;
  static constexpr uint32_t kInputPull = 0x8U;
  static constexpr uint32_t kOutputPushPull50MHz = 0x3U;
  static constexpr uint32_t kOutputOpenDrain50MHz = 0x7U;
  static constexpr uint32_t kShift = (PIN % 8U) * 4U;
  static constexpr uint32_t kMask = 1U << PIN;

  static F1GpioRegs* Regs() { return reinterpret_cast<F1GpioRegs*>(PORT); }
};

}  // namespace XrDap
