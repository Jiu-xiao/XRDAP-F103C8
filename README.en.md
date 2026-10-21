# XRDAP-F103C8

[中文](README.md)

CMSIS-DAP v2 debug probe firmware for STLINK V2 style STM32F103 boards, built on [libxr](https://github.com/xrobot-org/libxr) and part of the XR-DAP family (see also [XRDAP-ESP](https://github.com/Jiu-xiao/XRDAP-ESP)). Keil, pyOCD, probe-rs and OpenOCD use it as a standard CMSIS-DAP probe. The firmware also implements the XR vendor extension: one command reads or writes many addresses, follows pointer chains, and samples target variables on the probe at a fixed period, which the host tool [XRDAP-Host](https://github.com/Jiu-xiao/XRDAP-Host) (`xrdap`) shows and plots.

The whole firmware fits the 64 KB flash of an STM32F103C8.

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
| `PB6` | nRESET (open-drain output, released by default and held high by the board's pull-up; pulled low only when the debug tool asks for a reset) |
| `PA9` | Status light (red/blue, PA9 high lights red, low lights blue) |
| `PA11` / `PA12` | USB D- / D+ |

The STLINK V2 standard board usually has no pull-up on SWDIO; it relies on the target's pull-up, or an external pull-up between SWDIO and 3.3V. Powering or replugging the probe does not reset the target.

### Status light

The colours follow the status light rule shared by the XR-DAP family ([XRDAP-Core](https://github.com/Jiu-xiao/XRDAP-Core)): red for a wired (USB) link, blue for a wireless link and purple for no link. This probe has USB only; its status light shows the states below. "Connected" and "running" are reported by the debug tool through DAP_HostStatus and are cleared by DAP_Disconnect or after 5 seconds without any DAP command.

| State | Light |
|---|---|
| Power-up | one sweep between red and blue |
| USB not yet configured by a computer (for example on a power supply only) | solid purple |
| USB connected, idle | solid red |
| Debug tool connected | red and purple back and forth every 2 seconds |
| Target running | red and blue back and forth every second |
| Every DAP command | off for about 30 ms, about 15 times a second under steady traffic |

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

Once USB is plugged in, the probe enumerates as `1D50:6199` with the product name `XR-DAP-F103 CMSIS-DAP`. Its serial number is `XRobot-XRDAP-` followed by the chip UID in upper-case hex (written `XRobot-XRDAP-<UID>` below), the same serial that DAP_Info reports. The DAP_Info firmware version is the version number plus the commit it was built from, for example `0.6.0+g1a2b3c4`. On Linux, access as a normal user needs a udev rule:

```
SUBSYSTEM=="usb", ATTR{idVendor}=="1d50", ATTR{idProduct}=="6199", MODE="0666"
```

## Usage

Debug tools use it as a CMSIS-DAP v2 probe, for example OpenOCD:

```
openocd -f interface/cmsis-dap.cfg -c "adapter serial XRobot-XRDAP-<UID>" -f target/<target>.cfg
```

`xrdap list` lists the probes:

```
$ xrdap list
XRobot-XRDAP-<UID>  XR-DAP-F103 CMSIS-DAP
    USB XRobot-XRDAP-<UID>    --probe usb:XRobot-XRDAP-<UID>
```

`xrdap info` shows DAP_Info, the capabilities of the vendor extension, and the target's DPIDR and core (only the probe part is shown below; `xrdap` prints Chinese text):

```
$ xrdap info -p <last digits of the UID>
连接：USB XRobot-XRDAP-<UID>
探针：XRobot XR-DAP-F103，序列号 XRobot-XRDAP-<UID>，固件 0.6.0+g<commit>
CMSIS-DAP：2.0.0，包长 512 字节，最多 4 个包在途
厂商扩展：版本 1，功能 READ、WRITE、采样、8 字节访问（0x0F）
  每次最多 32 个条目，最多 8 级指针，采样缓冲 4096 字节
```

`xrdap read` and `write` access target memory by variable name from an ELF built with `-g`. The target program `app.elf` below has a global `counter` and a struct `state`:

```
$ xrdap read -p <last digits of the UID> --elf app.elf counter state
counter        = 16342240 (0x00F95CE0)
state.ticks    = 21079 (0x00005257)
state.wave     = 0.6699997
state.step     = 1 (0x0001)
state.flags[0] = 1 (0x01)
state.flags[1] = 2 (0x02)
state.flags[2] = 3 (0x03)
state.flags[3] = 4 (0x04)

$ xrdap write -p <last digits of the UID> --elf app.elf 'state.flags[2]=9'
state.flags[2] ← 9，读回 9 (0x09)
```

`xrdap watch` samples on the probe at a fixed period; `@10` samples a variable every 10th tick. The run below samples for 2 seconds at 1 ms:

```
$ xrdap watch -p <last digits of the UID> --elf app.elf counter state.wave state.ticks@10 --period 1ms --duration 2 --csv w.csv
已停止：2001 帧，用时 2.00 s；探针跳过节拍 0，缓冲丢帧 0，帧序号缺口 0，读取失败的值 0
```

Each row of `w.csv` is one frame: time, tick number and the value of each variable; cells not due in that tick are empty.

## Vendor extension

The vendor extension code comes from [XRDAP-Core](https://github.com/Jiu-xiao/XRDAP-Core); see commands 0x80 to 0x85 in section 3 of its [protocol document](https://github.com/Jiu-xiao/XRDAP-Core/blob/main/docs/protocol.en.md). This firmware differs from the ESP version as follows:

- USB is the only transport, and there is no 0x86 Wi-Fi command. Sampling belongs to the USB debug session, which ends on DAP_Disconnect or a USB bus reset (a replug, or the computer resetting the device); sampling stops with it.
- The sampling buffer is 4096 bytes (limited by the 20 KB of RAM), with at most 32 items per frame.
- TIM2 makes the ticks (1 us resolution) and the main loop runs the sampling: after each tick it handles one pending DAP command, then checks whether TIM2 counted another tick.

## Building

Building uses GCC ([Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) 14.2), CMake and Ninja:

```
git clone --recursive https://github.com/Jiu-xiao/XRDAP-F103C8.git
cd XRDAP-F103C8
cmake --preset Release
cmake --build build/Release
```

Each of `build/Release/XRDAP_F103C8_clock_72m/` and `build/Release/XRDAP_F103C8_clock_96m/` then holds an `XRDAP-F103C8.elf`.

The hardware-independent parts (for now the colour conversion of the status light) have host tests built with the system compiler:

```
cmake -S test -B build-test
cmake --build build-test
ctest --test-dir build-test
```

| Path | Contents |
|---|---|
| `User/app_main.cpp` | USB device, DAP instance and main loop |
| `User/tick_timer.*` | Sampling tick timer (TIM2) |
| `User/probe_info.*` | Probe identity (DAP_Info, USB serial) |
| `User/status_led.*` | Status light (PA9, TIM1) |
| `User/led_drive.hpp` | One frame of the status light rule to on/off and red share of this two-colour light, without hardware |
| `test/` | Host tests |
| `Middlewares/Third_Party/XRDAP-Core` | XRDAP-Core submodule: XR vendor extension, session and command execution, status light rule, probe identity, register-level GPIO for SWD |
| `Core/`, `Drivers/` | STM32CubeMX-generated initialization code and HAL |
| `Middlewares/Third_Party/LibXR` | libxr submodule |
