#include "main.h"

#include <stdbool.h>

#include "app_bridge.h"
#include "app_config.h"
#include "board.h"
#include "ethernetif.h"
#include "lwip/init.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "vofa_firewater.h"
#include "udp_transport.h"

static struct netif network_interface;

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

static void network_init(void) {
  ip4_addr_t address;
  ip4_addr_t netmask;
  ip4_addr_t gateway;

  IP4_ADDR(&address, APP_IP_ADDR0, APP_IP_ADDR1, APP_IP_ADDR2, APP_IP_ADDR3);
  IP4_ADDR(&netmask, APP_NETMASK_ADDR0, APP_NETMASK_ADDR1,
           APP_NETMASK_ADDR2, APP_NETMASK_ADDR3);
  IP4_ADDR(&gateway, APP_GATEWAY_ADDR0, APP_GATEWAY_ADDR1,
           APP_GATEWAY_ADDR2, APP_GATEWAY_ADDR3);

  lwip_init();
  if (netif_add(&network_interface, &address, &netmask, &gateway, NULL,
                ethernetif_init, ethernet_input) == NULL) {
    Error_Handler();
  }
  netif_set_default(&network_interface);
  ethernet_link_check_state(&network_interface);
}

int main(void) {
  uint32_t last_link_check = 0U;
  uint32_t last_heartbeat = 0U;

  HAL_Init();
  system_clock_config();
  if (!board_peripherals_init() || !board_peripherals_start()) {
    Error_Handler();
  }
  app_bridge_init();
  board_phy_reset_release();
  network_init();
  if (!udp_transport_init()) {
    Error_Handler();
  }

  while (1) {
    uint32_t now = HAL_GetTick();
    app_bridge_poll(now);
    ethernetif_input(&network_interface);
    sys_check_timeouts();
    now = HAL_GetTick();
    udp_transport_poll(now);
    vofa_firewater_poll(now);

    if ((uint32_t)(now - last_link_check) >= 100U) {
      last_link_check = now;
      ethernet_link_check_state(&network_interface);
      board_led_set(BOARD_LED_NETWORK,
                    netif_is_link_up(&network_interface) != 0);
    }
    if ((uint32_t)(now - last_heartbeat) >= 500U) {
      last_heartbeat = now;
      board_led_toggle(BOARD_LED_STATUS);
    }
  }
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
