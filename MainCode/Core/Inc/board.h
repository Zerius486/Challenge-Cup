#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_frame.h"
#include "stm32f4xx_hal.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart6_rx;

typedef enum {
  BOARD_LED_STATUS = 0,
  BOARD_LED_NETWORK,
  BOARD_LED_FAULT
} BoardLed;

enum {
  BOARD_PERIPHERAL_NONE = 0U,
  BOARD_PERIPHERAL_CAN1 = 1U << 0,
  BOARD_PERIPHERAL_CAN2 = 1U << 1,
  BOARD_PERIPHERAL_FORCE_UART = 1U << 2,
  BOARD_PERIPHERAL_VOFA_UART = 1U << 3,
  BOARD_PERIPHERAL_ALL = BOARD_PERIPHERAL_CAN1 | BOARD_PERIPHERAL_CAN2 |
                         BOARD_PERIPHERAL_FORCE_UART | BOARD_PERIPHERAL_VOFA_UART
};

bool board_peripherals_init(void);
bool board_peripherals_start(void);
bool board_peripherals_init_selected(uint32_t peripherals);
bool board_peripherals_start_selected(uint32_t peripherals);
void board_phy_reset_release(void);
void board_led_set(BoardLed led, bool on);
void board_led_toggle(BoardLed led);
uint32_t board_critical_enter(void);
void board_critical_exit(uint32_t state);
void board_delay_ms(uint32_t delay_ms);
bool board_can_send(uint8_t bus, const CanFrame *frame, uint32_t timeout_ms);
bool board_force_uart_start_rx(uint8_t *buffer, uint16_t length);
bool board_force_uart_maintain_rx(uint8_t *buffer, uint16_t length,
                                  bool *restarted);
uint16_t board_force_uart_rx_index(uint16_t length);
bool board_force_uart_send(const uint8_t *data, uint16_t length,
                           uint32_t timeout_ms);
bool board_vofa_uart_send(const uint8_t *data, uint16_t length,
                          uint32_t timeout_ms);

#endif
