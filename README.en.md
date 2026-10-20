# XRDAP-F103C8

[中文](README.md)

CMSIS-DAP v2 debug probe firmware for STLINK V2 style STM32F103 boards, built on [libxr](https://github.com/xrobot-org/libxr) and part of the XR-DAP family (see also [XRDAP-ESP](https://github.com/Jiu-xiao/XRDAP-ESP)). Keil, pyOCD, probe-rs and OpenOCD use it as a standard CMSIS-DAP probe. The firmware also implements the XR vendor extension: one command reads or writes many addresses, follows pointer chains, and samples target variables on the probe at a fixed period, which the host tool [XRDAP-Host](https://github.com/Jiu-xiao/XRDAP-Host) (`xrdap`) shows and plots.

The whole firmware is about 53 KB and fits the 64 KB flash of an STM32F103C8.

## Hardware

### STLINK V2 standard board
![STLINK V2 standard board](imgs/stlink2.png)

### STLINK V2 aluminium-case board
![STLINK V2 aluminium-case board](imgs/stlink1.png)

The probe MCU is an `STM32F103C8` or a pin-compatible F103 part (for example `APM32F103C8`) with at least 64 KB of flash.

| Pin | Function |
|---|---|
| `PB13` | SWCLK |
| `PB14` | SWDIO |
| `PB6` | nRESET |
| `PA9` | LED (lights up with DAP activity) |
| `PA11` / `PA12` | USB D- / D+ |

The STLINK V2 standard board usually has no pull-up on SWDIO; it relies on the target's pull-up, or an external pull-up between SWDIO and 3.3V.

## Flashing the firmware

The release holds two images that differ only in the system clock:

| Image | System clock | For |
|---|---|---|
| `XRDAP-F103C8-72m.bin` | 72 MHz | STM32F103C8 |
| `XRDAP-F103C8-96m.bin` | 96 MHz | APM32F103C8 and other compatible parts that run above 72 MHz |

The firmware starts at `0x08000000` and is written through the board's own SWD port (the SWCLK, SWDIO, GND and 3.3V pads) with another debug probe, for example a J-Link:

```
openocd -f interface/jlink.cfg -c "transport select swd" -f target/stm32f1x.cfg -c "program XRDAP-F103C8-72m.bin 0x08000000 verify reset exit"
```

Once USB is plugged in, the probe enumerates as `1D50:6199` with the product name `XR-DAP-F103 CMSIS-DAP`. Its serial number is `XRobot-XRDAP-` followed by the chip UID in upper-case hex, the same serial that DAP_Info reports. On Linux, access as a normal user needs a udev rule:

```
SUBSYSTEM=="usb", ATTR{idVendor}=="1d50", ATTR{idProduct}=="6199", MODE="0666"
```

## Usage

Debug tools use it as a CMSIS-DAP v2 probe, for example OpenOCD:

```
openocd -f interface/cmsis-dap.cfg -c "adapter serial XRobot-XRDAP-21001200142D363637365744" -f target/stm32f3x.cfg
```

`xrdap list` lists the probes:

```
$ xrdap list
XRobot-XRDAP-21001200142D363637365744  XR-DAP-F103 CMSIS-DAP
    USB XRobot-XRDAP-21001200142D363637365744    --probe usb:XRobot-XRDAP-21001200142D363637365744
```

`xrdap info` shows DAP_Info and the capabilities of the vendor extension (output without a target attached; `xrdap` prints Chinese text):

```
$ xrdap info -p 21001200142D363637365744
连接：USB XRobot-XRDAP-21001200142D363637365744
探针：XRobot XR-DAP-F103，序列号 XRobot-XRDAP-21001200142D363637365744，固件 0.6.0
CMSIS-DAP：2.0.0，包长 512 字节，最多 4 个包在途
厂商扩展：版本 1，功能 READ、WRITE、采样、8 字节访问（0x0F）
  每次最多 32 个条目，最多 8 级指针，采样缓冲 4096 字节
  探针时间 128.435740 s，单次 SWD 访问耗时还没有标定（每次 SUB_SET 时按当前时钟标定）
目标：连接失败（读 DPIDR 失败：SWD 应答 没有应答（检查接线、目标供电）（响应 05 00 07））
```

Reading and writing variables and sampling (`xrdap read`, `write`, `watch`) are described in XRDAP-Host.

## Vendor extension

The command format is the same as in XRDAP-ESP; see commands 0x80 to 0x85 in section 3 of the [XRDAP-ESP protocol document](https://github.com/Jiu-xiao/XRDAP-ESP/blob/main/docs/protocol.en.md). This firmware differs from the ESP version as follows:

- USB is the only transport. Sampling stops after a USB bus reset or unplug, and there is no 0x86 Wi-Fi command.
- The sampling buffer is 4096 bytes (limited by the 20 KB of RAM), with at most 32 items per frame.
- The main loop runs the sampling: after each tick it handles one pending DAP command, then checks whether the next tick is due. The frame time reported by SUB_SET is the trial-read time multiplied by 1.6.

## Building

Building uses GCC ([Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) 14.2), CMake and Ninja:

```
git clone --recursive https://github.com/Jiu-xiao/XRDAP-F103C8.git
cd XRDAP-F103C8
cmake --preset Release
cmake --build build/Release
```

Each of `build/Release/XRDAP_F103C8_clock_72m/` and `build/Release/XRDAP_F103C8_clock_96m/` then holds an `XRDAP-F103C8.elf`.

| Path | Contents |
|---|---|
| `User/app_main.cpp` | USB device, DAP instance and main loop |
| `User/fast_gpio.hpp` | Register-level GPIO for the SWD bit-bang loop |
| `User/probe_info.*` | Probe identity (DAP_Info, USB serial) |
| `User/vendor/` | XR vendor extension: command handling, MEM-AP access, sampling, frame buffer |
| `Core/`, `Drivers/` | STM32CubeMX-generated initialization code and HAL |
| `Middlewares/Third_Party/LibXR` | libxr submodule |
