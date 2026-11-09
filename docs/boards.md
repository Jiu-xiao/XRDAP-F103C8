# 板子

[English](boards.en.md)

本文说明 XRDAP-F103C8 的两种系统时钟、两种 Flash 大小和状态灯，引脚见 [README](../README.md)。探针是 USB 全速设备 `1D50:6199`，bcdDevice 为 `0x0400`（XRDAP-Core 的 `xrdap/probe/family.hpp`），只有一个 CMSIS-DAP v2 接口；DAP 包长 512 字节，DAP 包数为 4（DAP_Info 报告），采样缓冲 4096 字节（厂商扩展的 INFO 报告）。

## 系统时钟

两种固件都按 8 MHz 晶振设置时钟：

| 固件 | 系统时钟 | USB 时钟 |
|---|---|---|
| `xrdap-f103c8-72m` | 8 MHz × 9 = 72 MHz | 72 MHz ÷ 1.5 = 48 MHz |
| `xrdap-f103c8-96m` | 8 MHz × 12 = 96 MHz | 96 MHz ÷ 2 = 48 MHz |

`xrdap-f103cb-72m`、`xrdap-f103cb-96m` 的时钟设置分别与 `xrdap-f103c8-72m`、`xrdap-f103c8-96m` 相同。

STM32F103 的最高主频为 72 MHz，USB 分频系数只有 1 和 1.5，因此使用 72 MHz 的固件。APM32F103 可以运行在 96 MHz，USB 分频位扩展为两位（RCC_CFGR 的 USBDPSC[23:22]），可以二分频，因此两种固件都可以使用。时钟设置在 `Core/Src/main.c` 的 `XR_ApplyClockProfile()` 中。

SWD 由 GPIO 软件产生，时序按 CPU 周期标定（`User/app_main.cpp` 中的 `kSwdTiming`），SWD 的速度与系统时钟成正比。

## Flash 大小

`xrdap-f103c8-*` 按 64 KB Flash 编译，C 和 C++ 代码都为 `-Os`。`xrdap-f103cb-*` 按 128 KB Flash 编译（CMake 选项 `XR_FLASH_SIZE_KB=128`），C 代码为 `-Os`，C++ 代码为 `-O3`，处理 DAP 命令更快，镜像约 100 KB，需要 Flash 大小寄存器 `0x1FFFF7E0` 读出 128 KB 的芯片。

下表是 0.6.0 固件下 OpenOCD 0.12.0 经本探针（SWD，`adapter speed 30000`）向 STM32H723 的 SRAM（`0x24000000`）写入 128 KiB（`load_image`）和读出 128 KiB（`dump_image`）的用时：

| 固件 | 写入 | 读出 |
|---|---|---|
| `xrdap-f103c8-72m` | 0.62 s | 0.62 s |
| `xrdap-f103cb-72m` | 0.48 s | 0.47 s |
| `xrdap-f103cb-96m` | 0.36 s | 0.36 s |

## 状态灯

PA9 上反向并联着一红一蓝两颗 LED：PA9 为高电平时亮红，为低电平时亮蓝，为输入时两颗都熄灭。PA9 是 TIM1_CH2 的 PWM 输出，占空比为红色所占的比例，红蓝各半时显示紫色。灯色按 XR-DAP 全系列共用的状态灯规则（XRDAP-Core README 的“状态灯”一节）：

| 状态 | 灯 |
|---|---|
| 上电 | 在红、蓝之间扫过一次，用时 0.8 s |
| USB 还没有被电脑配置（例如只接了电源） | 紫色常亮 |
| USB 已被电脑配置，空闲 | 红色常亮 |
| 调试工具已连接 | 红、紫之间往返，周期 2 s |
| 目标运行中 | 红、蓝之间往返，周期 1 s |
| 每条 DAP 命令 | 熄灭 33 ms；命令连续时每 66 ms 熄灭一次 |

“已连接”和“运行中”由调试工具经 DAP_HostStatus 报告，收到 DAP_Disconnect 或 5 s 内没有任何 DAP 命令时清除。
