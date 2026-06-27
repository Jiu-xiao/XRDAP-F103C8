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

/* No DMA Resources generated. */

static bool XR_ConfigTim1Ch2Pwm(TIM_HandleTypeDef* htim, uint32_t target_freq) {
  if (target_freq == 0u) {
    return false;
  }

  uint32_t clock_freq = HAL_RCC_GetPCLK2Freq();
#ifdef RCC_CFGR_PPRE2
  if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) {
    clock_freq *= 2u;
  }
#endif

  uint32_t prescaler = 1u;
  uint32_t period = 0u;
  bool found = false;

  for (prescaler = 1u; prescaler <= 0xFFFFu; ++prescaler) {
    uint32_t temp_period = clock_freq / (prescaler * target_freq);
    if (temp_period > 0u && temp_period <= 0x10000u) {
      period = temp_period;
      found = true;
      break;
    }
  }

  if (!found || period == 0u) {
    return false;
  }

  htim->Init.Prescaler = prescaler - 1u;
  htim->Init.Period = period - 1u;
  if (HAL_TIM_PWM_Init(htim) != HAL_OK) {
    return false;
  }

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0u;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  return HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_2) == HAL_OK;
}

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
  /* Terminal Configuration */

  // clang-format on
  // NOLINTEND
  /* User Code Begin 3 */
  static constexpr auto USB_FS_LANG_PACK =
      LibXR::USB::DescriptorStrings::MakeLanguagePack(
          LibXR::USB::DescriptorStrings::Language::EN_US, "XRobot", "CMSIS-DAP",
          "XRUSB-DEMO-XRDAP-");

  LibXR::USB::CDCUart usb_fs_cdc(128, 128, 3);
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
  if (!XR_ConfigTim1Ch2Pwm(&htim1, 10000u)) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  while (true) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
                          daplink.EpInBusy() ? (htim1.Init.Period + 1u) : 0u);

    LibXR::Thread::Sleep(1);
  }
  /* User Code End 3 */
}
