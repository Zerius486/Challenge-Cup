#include "board.h"

#include <string.h>

#define MOCK_CAN_CAPACITY 128U
#define MOCK_UART_TX_CAPACITY 64U

static CanFrame can_attempts[MOCK_CAN_CAPACITY];
static uint8_t can_buses[MOCK_CAN_CAPACITY];
static size_t can_attempt_count;
static size_t can_success_count;
static size_t can_fail_count;
static uint8_t *force_rx_buffer;
static uint16_t force_rx_length;
static uint16_t force_rx_write_index;
static bool force_rx_faulted;
static uint8_t uart_tx_buffer[MOCK_UART_TX_CAPACITY];
static size_t uart_tx_length;
static size_t delay_count;
static uint32_t delay_total_ms;

void mock_board_reset(void) {
  memset(can_attempts, 0, sizeof(can_attempts));
  memset(can_buses, 0, sizeof(can_buses));
  can_attempt_count = 0U;
  can_success_count = 0U;
  can_fail_count = 0U;
  force_rx_buffer = NULL;
  force_rx_length = 0U;
  force_rx_write_index = 0U;
  force_rx_faulted = false;
  memset(uart_tx_buffer, 0, sizeof(uart_tx_buffer));
  uart_tx_length = 0U;
  delay_count = 0U;
  delay_total_ms = 0U;
}

void mock_board_fail_next_can(size_t count) {
  can_fail_count = count;
}

uint32_t board_critical_enter(void) {
  return 0U;
}

void board_critical_exit(uint32_t state) {
  (void)state;
}

void board_delay_ms(uint32_t delay_ms) {
  ++delay_count;
  delay_total_ms += delay_ms;
}

bool board_can_send(uint8_t bus, const CanFrame *frame, uint32_t timeout_ms) {
  (void)timeout_ms;
  if (frame == NULL || can_attempt_count >= MOCK_CAN_CAPACITY) {
    return false;
  }
  can_attempts[can_attempt_count] = *frame;
  can_buses[can_attempt_count] = bus;
  ++can_attempt_count;
  if (can_fail_count > 0U) {
    --can_fail_count;
    return false;
  }
  ++can_success_count;
  return true;
}

size_t mock_board_can_attempt_count(void) {
  return can_attempt_count;
}

size_t mock_board_can_success_count(void) {
  return can_success_count;
}

const CanFrame *mock_board_can_attempt(size_t index, uint8_t *bus) {
  if (index >= can_attempt_count) {
    return NULL;
  }
  if (bus != NULL) {
    *bus = can_buses[index];
  }
  return &can_attempts[index];
}

bool board_force_uart_start_rx(uint8_t *buffer, uint16_t length) {
  if (buffer == NULL || length == 0U) {
    return false;
  }
  force_rx_buffer = buffer;
  force_rx_length = length;
  force_rx_write_index = 0U;
  force_rx_faulted = false;
  return true;
}

bool board_force_uart_maintain_rx(uint8_t *buffer, uint16_t length,
                                  bool *restarted) {
  if (buffer == NULL || buffer != force_rx_buffer || length == 0U ||
      length != force_rx_length || restarted == NULL) {
    return false;
  }
  *restarted = force_rx_faulted;
  if (force_rx_faulted) {
    force_rx_write_index = 0U;
    force_rx_faulted = false;
  }
  return true;
}

uint16_t board_force_uart_rx_index(uint16_t length) {
  return length == force_rx_length ? force_rx_write_index : 0U;
}

bool mock_board_inject_force(const uint8_t *data, size_t length) {
  if (data == NULL || force_rx_buffer == NULL || length >= force_rx_length) {
    return false;
  }
  for (size_t i = 0U; i < length; ++i) {
    force_rx_buffer[force_rx_write_index] = data[i];
    force_rx_write_index =
        (uint16_t)((force_rx_write_index + 1U) % force_rx_length);
  }
  return true;
}

void mock_board_fault_force_rx(void) {
  force_rx_faulted = true;
}

bool board_force_uart_send(const uint8_t *data, uint16_t length,
                           uint32_t timeout_ms) {
  (void)timeout_ms;
  if (data == NULL || length == 0U || length > sizeof(uart_tx_buffer)) {
    return false;
  }
  memcpy(uart_tx_buffer, data, length);
  uart_tx_length = length;
  return true;
}

size_t mock_board_uart_tx(const uint8_t **data) {
  if (data != NULL) {
    *data = uart_tx_buffer;
  }
  return uart_tx_length;
}

size_t mock_board_delay_count(void) {
  return delay_count;
}

uint32_t mock_board_delay_total_ms(void) {
  return delay_total_ms;
}
