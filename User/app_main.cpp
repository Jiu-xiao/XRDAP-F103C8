#include "app_main.h"

#include "cdc_uart.hpp"
#include "libxr.hpp"
#include "main.h"
#include "stm32_adc.hpp"
#include "stm32_can.hpp"
#include "stm32_canfd.hpp"
#include "stm32_dac.hpp"
#include "stm32_flash.hpp"
#include "stm32_gpio.hpp"
#include "stm32_i2c.hpp"
#include "stm32_power.hpp"
#include "stm32_pwm.hpp"
#include "stm32_spi.hpp"
#include "stm32_timebase.hpp"
#include "stm32_uart.hpp"
#include "stm32_usb_dev.hpp"
#include "stm32_watchdog.hpp"
#include "flash_map.hpp"

using namespace LibXR;

/* User Code Begin 1 */
#include "cdc_to_uart.hpp"
#include "daplink_v2.hpp"
#include "debug/swd_general_gpio.hpp"

static uint8_t usb_fs_ep0_in_buf[64];
static uint8_t usb_fs_ep0_out_buf[64];
static uint8_t usb_fs_ep1_in_buf[512];
static uint8_t usb_fs_ep1_out_buf[512];
static uint8_t usb_fs_ep2_in_buf[1024];
static uint8_t usb_fs_ep2_out_buf[1024];
static uint8_t usb_fs_ep3_in_buf[128];
static uint8_t usb_fs_ep3_out_buf[128];
/* User Code End 1 */
// NOLINTBEGIN
// clang-format off
/* External HAL Declarations */
extern PCD_HandleTypeDef hpcd_USB_FS;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;

/* DMA Resources */
static uint8_t usart2_tx_buf[2048];
static uint8_t usart2_rx_buf[2048];

extern "C" void app_main(void) {
  // clang-format on
  // NOLINTEND
  /* User Code Begin 2 */
  
  /* User Code End 2 */
  // clang-format off
  // NOLINTBEGIN
  STM32Timebase timebase;
  PlatformInit();
  STM32PowerManager power_manager;

  /* GPIO Configuration */
  STM32GPIO SWCLK(SWCLK_GPIO_Port, SWCLK_Pin);
  STM32GPIO SWDIO(SWDIO_GPIO_Port, SWDIO_Pin);
  STM32GPIO NRST(NRST_GPIO_Port, NRST_Pin);


  STM32PWM pwm_tim1_ch2(&htim1, TIM_CHANNEL_2, false);

  STM32UART usart2(&huart2,
              usart2_rx_buf, usart2_tx_buf, 5);

  /* Terminal Configuration */

  // clang-format on
  // NOLINTEND
  /* User Code Begin 3 */
  static constexpr auto USB_FS_LANG_PACK =
      LibXR::USB::DescriptorStrings::MakeLanguagePack(
          LibXR::USB::DescriptorStrings::Language::EN_US, "XRobot", "CMSIS-DAP",
          "XRUSB-DEMO-XRDAP-");

  LibXR::USB::CDCToUart usb_fs_cdc(usart2, 128, 128, 3);
  LibXR::Debug::SwdGeneralGPIO swd_gpio(SWCLK, SWDIO, 7);
  LibXR::USB::DapLinkV2Class daplink(swd_gpio, &NRST);

  STM32USBDeviceDevFs usb_fs(&hpcd_USB_FS,
                             {{usb_fs_ep0_in_buf, usb_fs_ep0_out_buf, 64, 64},
                              {usb_fs_ep1_in_buf, usb_fs_ep1_out_buf, 64, 64},
                              {usb_fs_ep2_in_buf, usb_fs_ep2_out_buf, 64, 64},
                              {usb_fs_ep3_in_buf, usb_fs_ep3_out_buf, 64, 64}},
                             USB::DeviceDescriptor::PacketSize0::SIZE_64,
                             0x0D28, 0x2040, 0x0201, {&USB_FS_LANG_PACK},
                             {{&daplink, &usb_fs_cdc}},
                             {reinterpret_cast<void *>(UID_BASE), 12});
  usb_fs.Init(false);
  usb_fs.Start(false);
  float busy = 0.0f;
  pwm_tim1_ch2.SetConfig({.frequency = 10000});
  pwm_tim1_ch2.SetDutyCycle(0.0f);
  pwm_tim1_ch2.Enable();
  static constexpr float step = 0.1f;
  while (true) {
    if (daplink.EpInBusy()) {
      busy = busy * (1.0f - step) + 1.0f * step;
    } else {
      busy = busy * (1.0f - step);
    }

    pwm_tim1_ch2.SetDutyCycle(busy);

    LibXR::Thread::Sleep(1);
  }
  /* User Code End 3 */
}