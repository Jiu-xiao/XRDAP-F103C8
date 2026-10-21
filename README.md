# XRDAP-F103C8

[English](README.en.md)

跑在 STLINK V2 形态 STM32F103 板子上的 CMSIS-DAP v2 调试探针固件，基于 [libxr](https://github.com/xrobot-org/libxr)，属于 XR-DAP 系列（另见 [XRDAP-ESP](https://github.com/Jiu-xiao/XRDAP-ESP)）。Keil、pyOCD、probe-rs、OpenOCD 把它当作普通的 CMSIS-DAP 探针使用。固件还实现了 XR 厂商扩展：一条命令读写多个地址、沿指针链解引用、在探针上按固定周期采样目标变量，配合电脑端工具 [XRDAP-Host](https://github.com/Jiu-xiao/XRDAP-Host)（`xrdap`）查看和画曲线。

整个固件约 56 KB，64 KB Flash 的 STM32F103C8 即可容纳。

## 硬件

### STLINK V2 标准版
![STLINK V2标准版](imgs/stlink2.png)

### STLINK V2 淘宝铝壳版
![STLINK V2淘宝铝壳版](imgs/stlink1.png)

探针 MCU 为 `STM32F103C8` 或引脚兼容的 F103 型号（例如 `APM32F103C8`），Flash 不少于 64 KB。

| 引脚 | 功能 |
|---|---|
| `PB13` | SWCLK |
| `PB14` | SWDIO |
| `PB6` | nRESET |
| `PA9` | 状态灯（红蓝双色，PA9 高电平亮红、低电平亮蓝） |
| `PA11` / `PA12` | USB D- / D+ |

STLINK V2 标准版的 SWDIO 上通常没有上拉电阻，需要目标板上拉，或在 SWDIO 与 3.3V 之间外加上拉电阻。

### 状态灯

灯色按 XR-DAP 全系列共用的状态灯规则（[XRDAP-Core](https://github.com/Jiu-xiao/XRDAP-Core)）：红色表示有线（USB）连接，蓝色表示无线连接，紫色表示没有连接。本探针只有 USB，状态灯如下。其中“已连接”“运行中”由调试工具通过 DAP_HostStatus 报告，收到 DAP_Disconnect 或 5 秒内没有任何 DAP 命令时清除。

| 状态 | 灯 |
|---|---|
| 上电 | 红、蓝之间扫过一次 |
| USB 还没有被电脑配置（例如只接了电源） | 紫色常亮 |
| USB 已连接、空闲 | 红色常亮 |
| 调试工具已连接 | 红、紫之间往返，2 秒一次 |
| 目标运行中 | 红、蓝之间往返，1 秒一次 |
| 每条 DAP 命令 | 熄灭约 30 ms，命令连续时每秒约 15 次 |

## 烧录固件

Release 中有两个镜像，区别只在系统时钟：

| 镜像 | 系统时钟 | 适用 |
|---|---|---|
| `XRDAP-F103C8-72m.bin` | 72 MHz | STM32F103C8 |
| `XRDAP-F103C8-96m.bin` | 96 MHz | APM32F103C8 等可超频的兼容型号 |

固件从 `0x08000000` 开始，通过板子上的 SWD 调试口（SWCLK、SWDIO、GND、3.3V 四个焊盘）用另一个调试器烧录，例如 J-Link：

```
openocd -f interface/jlink.cfg -c "transport select swd" -f target/stm32f1x.cfg -c "program XRDAP-F103C8-72m.bin 0x08000000 verify reset exit"
```

烧录后插上 USB，探针以 `1D50:6199` 枚举，产品名 `XR-DAP-F103 CMSIS-DAP`，序列号为 `XRobot-XRDAP-` 加芯片 UID 的大写十六进制，与 DAP_Info 中的序列号相同。Linux 下普通用户访问需要 udev 规则：

```
SUBSYSTEM=="usb", ATTR{idVendor}=="1d50", ATTR{idProduct}=="6199", MODE="0666"
```

## 使用

调试工具按 CMSIS-DAP v2 使用即可，例如 OpenOCD：

```
openocd -f interface/cmsis-dap.cfg -c "adapter serial XRobot-XRDAP-21001200142D363637365744" -f target/stm32f3x.cfg
```

`xrdap list` 列出探针：

```
$ xrdap list
XRobot-XRDAP-21001200142D363637365744  XR-DAP-F103 CMSIS-DAP
    USB XRobot-XRDAP-21001200142D363637365744    --probe usb:XRobot-XRDAP-21001200142D363637365744
```

`xrdap info` 显示 DAP_Info、厂商扩展的能力和目标内核：

```
$ xrdap info -p 21001200142D363637365744
连接：USB XRobot-XRDAP-21001200142D363637365744
探针：XRobot XR-DAP-F103，序列号 XRobot-XRDAP-21001200142D363637365744，固件 0.6.0
CMSIS-DAP：2.0.0，包长 512 字节，最多 4 个包在途
厂商扩展：版本 1，功能 READ、WRITE、采样、8 字节访问（0x0F）
  每次最多 32 个条目，最多 8 级指针，采样缓冲 4096 字节
  探针时间 10.543958 s，单次 SWD 访问耗时还没有标定（每次 SUB_SET 时按当前时钟标定）
目标：DPIDR 0x2BA01477（ARM，设计者代码 0x23B，DPv1，部件号 0xBA），SWD 时钟 4000000 Hz
内核：Cortex-M4 r0p1，运行中（DHCSR 0x01010001）
```

`xrdap read`、`write` 按 ELF（带 `-g` 调试信息）里的变量名读写目标内存。下面的目标程序 `app.elf` 中有全局变量 `counter` 和结构体 `state`：

```
$ xrdap read -p 21001200142D363637365744 --elf app.elf counter state
counter        = 16342240 (0x00F95CE0)
state.ticks    = 21079 (0x00005257)
state.wave     = 0.6699997
state.step     = 1 (0x0001)
state.flags[0] = 1 (0x01)
state.flags[1] = 2 (0x02)
state.flags[2] = 3 (0x03)
state.flags[3] = 4 (0x04)

$ xrdap write -p 21001200142D363637365744 --elf app.elf 'state.flags[2]=9'
state.flags[2] ← 9，读回 9 (0x09)
```

`xrdap watch` 在探针上按固定周期采样，`@10` 表示每 10 拍采一次。下面以 1 ms 周期采样 2 秒：

```
$ xrdap watch -p 21001200142D363637365744 --elf app.elf counter state.wave state.ticks@10 --period 1ms --duration 2 --csv w.csv
已停止：2001 帧，用时 2.00 s；探针跳过节拍 0，缓冲丢帧 0，帧序号缺口 0，读取失败的值 0
w.csv 的 host_time 列是对到电脑时钟的 Unix 时间（秒），按 SUB_READ 的往返时间估计，误差约 ±163 µs
$ head -4 w.csv
host_time,t,sample_tick,counter,state.wave,state.ticks
1792500177.385438,0.000000,0,19881886,0.5799998,25644
1792500177.386398,0.000960,1,19882608,0.5699998,
1792500177.387392,0.001954,2,19883378,0.5599998,
```

## 厂商扩展

厂商扩展的代码来自 [XRDAP-Core](https://github.com/Jiu-xiao/XRDAP-Core)，命令格式见其[协议文档](https://github.com/Jiu-xiao/XRDAP-Core/blob/main/docs/protocol.md)第 3 节的 0x80 到 0x85。本固件与 ESP 版的差别：

- 只有 USB 一种传输，采样在 USB 总线复位或拔出后停止，没有 0x86 Wi-Fi 命令；
- 采样缓冲为 4096 字节（20 KB RAM 的限制），一帧最多 32 个条目；
- 采样由主循环执行：每读完一拍，主循环先处理一条待执行的 DAP 命令，再检查下一拍是否到期；SUB_SET 报告的单帧耗时为试读耗时乘 1.6。

## 构建

使用 GCC（[Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) 14.2）、CMake 和 Ninja：

```
git clone --recursive https://github.com/Jiu-xiao/XRDAP-F103C8.git
cd XRDAP-F103C8
cmake --preset Release
cmake --build build/Release
```

`build/Release/XRDAP_F103C8_clock_72m/` 和 `build/Release/XRDAP_F103C8_clock_96m/` 下各生成一个 `XRDAP-F103C8.elf`。

| 目录 | 内容 |
|---|---|
| `User/app_main.cpp` | USB 设备、DAP 实例和主循环 |
| `User/fast_gpio.hpp` | SWD 位翻转用的寄存器级 GPIO |
| `User/probe_info.*` | 探针身份（DAP_Info、USB 序列号） |
| `User/status_led.*` | 状态灯 |
| `Middlewares/Third_Party/XRDAP-Core` | XRDAP-Core 子模块：XR 厂商扩展、状态灯规则、探针身份 |
| `Core/`、`Drivers/` | STM32CubeMX 生成的初始化代码和 HAL |
| `Middlewares/Third_Party/LibXR` | libxr 子模块 |
