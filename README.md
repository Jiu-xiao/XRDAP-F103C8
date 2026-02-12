# XRDAP-F103C8 适配常见STLINK V2的DAPLink固件

## 测试过的设备

### STLINK V2标准版
![STLINK V2标准版](imgs/stlink2.png)
### STLINK V2淘宝铝壳版
![STLINK V2淘宝铝壳版](imgs/stlink1.png)

## 引脚定义

- PB6: nRESET
- PA12: USB_DP
- PA11: USB_DM
- PA9: LED
- PB14: SWDIO
- PB13: SWCLK
- PA2: UART_TX
- PA3: UART_RX

## MCU

- STM32F103C8
- APM32F103C8
- 其他兼容MCU理论上也可以，但未测试
