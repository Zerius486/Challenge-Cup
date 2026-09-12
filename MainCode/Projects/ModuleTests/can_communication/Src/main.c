#include <stdbool.h>
#include <stdint.h>

#include "app_bridge.h"
#include "board.h"
#include "can_frame.h"
#include "main.h"
#include "standalone_runtime.h"
#include "test_config.h"

volatile uint32_t g_can_tx_count[2];
volatile uint32_t g_can_rx_count[2];
volatile uint32_t g_can_error[2];
volatile uint32_t g_can_last_rx_ms[2];
volatile uint32_t g_can_last_id[2];
volatile uint8_t g_can_last_dlc[2];
volatile uint8_t g_can_last_data[2][8];
volatile uint8_t g_can_rx_valid[2];

static uint32_t next_tx_ms;
static uint8_t sequence;

static bool bus_selected(uint8_t bus) {
  return (CAN_TEST_BUS_MASK & (1U << (bus - 1U))) != 0U;
}

static bool send_test_frame(uint8_t bus) {
  CanFrame frame = {0};
  frame.id = bus == 1U ? CAN_TEST_ID_CAN1 : CAN_TEST_ID_CAN2;
  frame.dlc = 8U;
  frame.data[0] = 0xCAU;
  frame.data[1] = 0x4EU;
  frame.data[2] = bus;
  frame.data[3] = sequence;
  frame.data[4] = (uint8_t)(HAL_GetTick() >> 0U);
  frame.data[5] = (uint8_t)(HAL_GetTick() >> 8U);
  frame.data[6] = (uint8_t)(HAL_GetTick() >> 16U);
  frame.data[7] = (uint8_t)(HAL_GetTick() >> 24U);
  if (!board_can_send(bus, &frame, 5U)) {
    ++g_can_error[bus - 1U];
    return false;
  }
  ++g_can_tx_count[bus - 1U];
  return true;
}

void app_bridge_on_can_frame(uint8_t bus, const CanFrame *frame,
                             uint32_t now_ms) {
  uint8_t index;
  if (frame == NULL || (bus != 1U && bus != 2U) || !bus_selected(bus)) {
    return;
  }
  index = (uint8_t)(bus - 1U);
  g_can_last_id[index] = frame->id;
  g_can_last_dlc[index] = frame->dlc;
  for (uint8_t i = 0U; i < frame->dlc && i < 8U; ++i) {
    g_can_last_data[index][i] = frame->data[i];
  }
  g_can_last_rx_ms[index] = now_ms;
  g_can_rx_valid[index] = 1U;
  ++g_can_rx_count[index];
  board_led_toggle(BOARD_LED_NETWORK);
}

void app_bridge_on_can_error(uint8_t bus, uint32_t error) {
  if (bus == 1U || bus == 2U) {
    g_can_error[bus - 1U] = error;
    standalone_set_fault(true);
  }
}

int main(void) {
  if (CAN_TEST_BUS_MASK == 0U || (CAN_TEST_BUS_MASK & ~3U) != 0U ||
      CAN_TEST_PERIOD_MS == 0U || CAN_TEST_ID_CAN1 > 0x7FFU ||
      CAN_TEST_ID_CAN2 > 0x7FFU) {
    Error_Handler();
  }
  standalone_runtime_init(
      (CAN_TEST_BUS_MASK & 1U ? BOARD_PERIPHERAL_CAN1 : 0U) |
      (CAN_TEST_BUS_MASK & 2U ? BOARD_PERIPHERAL_CAN2 : 0U));
  next_tx_ms = HAL_GetTick();

  while (1) {
    uint32_t now_ms = HAL_GetTick();
    if ((int32_t)(now_ms - next_tx_ms) >= 0) {
      if (bus_selected(1U)) {
        (void)send_test_frame(1U);
      }
      if (bus_selected(2U)) {
        (void)send_test_frame(2U);
      }
      ++sequence;
      next_tx_ms = now_ms + CAN_TEST_PERIOD_MS;
    }
    standalone_set_fault(g_can_error[0] != 0U || g_can_error[1] != 0U);
    standalone_heartbeat_poll(now_ms);
  }
}
