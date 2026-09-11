#include "standalone_runtime.h"

#include "board.h"
#include "main.h"

static void system_clock_config(void) {
  RCC_OscInitTypeDef oscillator = {0};
  RCC_ClkInitTypeDef clocks = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  oscillator.HSEState = RCC_HSE_ON;
  oscillator.PLL.PLLState = RCC_PLL_ON;
  oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  oscillator.PLL.PLLM = 8U;
  oscillator.PLL.PLLN = 336U;
  oscillator.PLL.PLLP = RCC_PLLP_DIV2;
  oscillator.PLL.PLLQ = 7U;
  if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
    Error_Handler();
  }

  clocks.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clocks.APB1CLKDivider = RCC_HCLK_DIV4;
  clocks.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_GetREVID() >= 0x1001U) {
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}

void standalone_runtime_init(uint32_t peripherals) {
  HAL_Init();
  system_clock_config();
  if (!board_peripherals_init_selected(peripherals) ||
      !board_peripherals_start_selected(peripherals)) {
    Error_Handler();
  }
  board_led_set(BOARD_LED_STATUS, false);
  board_led_set(BOARD_LED_NETWORK, false);
  board_led_set(BOARD_LED_FAULT, false);
}

void standalone_heartbeat_poll(uint32_t now_ms) {
  static uint32_t last_toggle_ms;
  if ((uint32_t)(now_ms - last_toggle_ms) >= 500U) {
    last_toggle_ms = now_ms;
    board_led_toggle(BOARD_LED_STATUS);
  }
}

void standalone_set_fault(bool active) {
  board_led_set(BOARD_LED_FAULT, active);
}

void Error_Handler(void) {
  __disable_irq();
  board_led_set(BOARD_LED_FAULT, true);
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
  (void)file;
  (void)line;
  Error_Handler();
}
#endif
