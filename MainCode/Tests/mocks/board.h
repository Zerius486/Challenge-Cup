#ifndef TEST_BOARD_H
#define TEST_BOARD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_frame.h"

bool board_can_send(uint8_t bus, const CanFrame *frame, uint32_t timeout_ms);
bool board_force_uart_start_rx(uint8_t *buffer, uint16_t length);
bool board_force_uart_maintain_rx(uint8_t *buffer, uint16_t length,
                                  bool *restarted);
uint16_t board_force_uart_rx_index(uint16_t length);
bool board_force_uart_send(const uint8_t *data, uint16_t length,
                           uint32_t timeout_ms);
uint32_t board_critical_enter(void);
void board_critical_exit(uint32_t state);
void board_delay_ms(uint32_t delay_ms);

void mock_board_reset(void);
void mock_board_fail_next_can(size_t count);
size_t mock_board_can_attempt_count(void);
size_t mock_board_can_success_count(void);
const CanFrame *mock_board_can_attempt(size_t index, uint8_t *bus);
bool mock_board_inject_force(const uint8_t *data, size_t length);
void mock_board_fault_force_rx(void);
size_t mock_board_uart_tx(const uint8_t **data);
size_t mock_board_delay_count(void);
uint32_t mock_board_delay_total_ms(void);

#endif
