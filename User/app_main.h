#pragma once

// 固件入口：CubeMX 的 main() 在时钟、GPIO、USB 和 TIM1 初始化之后调用一次，不再返回。在
// 其中构造 DAP、USB 设备和状态灯，然后进入主循环。
//
// Firmware entry: CubeMX's main() calls it once after the clock, GPIO, USB and TIM1
// set-up, and it never returns. It constructs the DAP, the USB device and the status
// light, then runs the main loop.

#ifdef __cplusplus
extern "C"
{
#endif

  void app_main(void);

#ifdef __cplusplus
}
#endif
