#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "board.h"
#include "main.h"
#include "standalone_runtime.h"
#include "test_config.h"

volatile uint32_t g_uart_tx_count;
volatile uint32_t g_uart_rx_bytes;
volatile uint32_t g_uart_rx_lines;
volatile uint32_t g_uart_rx_errors;
volatile uint32_t g_uart_last_rx_ms;
volatile uint16_t g_uart_last_line_length;
volatile uint8_t g_uart_rx_valid;
volatile uint8_t g_uart_stale;

static uint8_t dma_buffer[UART_TEST_RX_BUFFER_SIZE];
static uint16_t read_index;
static uint16_t line_length;
static uint32_t next_tx_ms;
static char line_buffer[UART_TEST_LINE_SIZE];

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms) {
  return (int32_t)(now_ms - deadline_ms) >= 0;
}

static void send_test_line(uint32_t now_ms) {
  char line[48];
  int length = snprintf(line, sizeof(line), "UART_TEST,%lu\r\n",
                        (unsigned long)g_uart_tx_count);
  if (length <= 0 || (size_t)length >= sizeof(line) ||
      !board_force_uart_send((const uint8_t *)line, (uint16_t)length, 20U)) {
    ++g_uart_rx_errors;
    return;
  }
  ++g_uart_tx_count;
  next_tx_ms = now_ms + UART_TEST_PERIOD_MS;
}

static void consume_rx(uint32_t now_ms) {
  bool restarted;
  uint16_t write_index;

  if (!board_force_uart_maintain_rx(dma_buffer,
                                    (uint16_t)sizeof(dma_buffer), &restarted)) {
    ++g_uart_rx_errors;
    g_uart_rx_valid = 0U;
    return;
  }
  if (restarted) {
    read_index = 0U;
    line_length = 0U;
    g_uart_rx_valid = 0U;
  }

  write_index = board_force_uart_rx_index((uint16_t)sizeof(dma_buffer));
  while (read_index != write_index) {
    uint8_t byte = dma_buffer[read_index];
    read_index = (uint16_t)((read_index + 1U) % sizeof(dma_buffer));
    ++g_uart_rx_bytes;
    if (line_length + 1U < sizeof(line_buffer)) {
      line_buffer[line_length++] = (char)byte;
    } else {
      line_length = 0U;
      ++g_uart_rx_errors;
    }
    if (byte == '\n') {
      line_buffer[line_length] = '\0';
      g_uart_last_line_length = line_length;
      g_uart_last_rx_ms = now_ms;
      g_uart_rx_valid = 1U;
      ++g_uart_rx_lines;
      line_length = 0U;
      board_led_toggle(BOARD_LED_NETWORK);
    }
  }
}

int main(void) {
  standalone_runtime_init(BOARD_PERIPHERAL_FORCE_UART);
  if (UART_TEST_PERIOD_MS == 0U || UART_TEST_RX_BUFFER_SIZE < 2U ||
      UART_TEST_LINE_SIZE < 2U ||
      !board_force_uart_start_rx(dma_buffer,
                                 (uint16_t)sizeof(dma_buffer))) {
    Error_Handler();
  }

  next_tx_ms = HAL_GetTick();
  while (1) {
    uint32_t now_ms = HAL_GetTick();
    consume_rx(now_ms);
    if (time_reached(now_ms, next_tx_ms)) {
      send_test_line(now_ms);
    }
    g_uart_stale =
        g_uart_rx_valid == 0U ||
                (uint32_t)(now_ms - g_uart_last_rx_ms) >
                    UART_TEST_STALE_TIMEOUT_MS
            ? 1U
            : 0U;
    standalone_set_fault(g_uart_rx_errors != 0U || g_uart_stale != 0U);
    standalone_heartbeat_poll(now_ms);
  }
}
