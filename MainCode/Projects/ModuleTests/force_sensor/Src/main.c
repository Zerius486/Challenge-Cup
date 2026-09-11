#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "board.h"
#include "force_sensor.h"
#include "main.h"
#include "standalone_runtime.h"
#include "test_config.h"

volatile float g_force_fx;
volatile float g_force_fy;
volatile float g_force_fz;
volatile float g_force_mx;
volatile float g_force_my;
volatile float g_force_mz;
volatile uint32_t g_force_frame_count;
volatile uint32_t g_force_non_sample_frame_count;
volatile uint32_t g_force_parse_error_count;
volatile uint32_t g_force_io_error_count;
volatile uint32_t g_force_uart_restart_count;
volatile uint32_t g_force_last_sample_ms;
volatile uint8_t g_force_last_command;
volatile uint8_t g_force_valid;
volatile uint8_t g_force_stale;

typedef enum {
  SENSOR_WAIT_BEFORE_STOP = 0,
  SENSOR_WAIT_BEFORE_MODE,
  SENSOR_ACTIVE
} SensorSetupPhase;

static ForceSensorParser parser;
static uint8_t dma_buffer[APP_FORCE_RX_DMA_SIZE];
static uint16_t read_index;
static uint32_t setup_deadline_ms;
static uint32_t next_request_ms;
static uint32_t active_since_ms;
static bool uart_rx_healthy = true;
static SensorSetupPhase setup_phase;

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms) {
  return (int32_t)(now_ms - deadline_ms) >= 0;
}

static bool send_simple_command(ForceSensorCommand command) {
  uint8_t bytes[5];
  size_t length = force_sensor_build_simple_command(command, bytes);
  if (length != sizeof(bytes) ||
      !board_force_uart_send(bytes, (uint16_t)length, 10U)) {
    ++g_force_io_error_count;
    return false;
  }
  g_force_last_command = (uint8_t)command;
  return true;
}

static void store_sample(const ForceSensorSample *sample, uint32_t now_ms) {
  if (!isfinite(sample->fx) || !isfinite(sample->fy) ||
      !isfinite(sample->fz) || !isfinite(sample->mx) ||
      !isfinite(sample->my) || !isfinite(sample->mz)) {
    ++g_force_parse_error_count;
    return;
  }
  g_force_fx = sample->fx;
  g_force_fy = sample->fy;
  g_force_fz = sample->fz;
  g_force_mx = sample->mx;
  g_force_my = sample->my;
  g_force_mz = sample->mz;
  g_force_last_sample_ms = now_ms;
  g_force_valid = 1U;
  g_force_stale = 0U;
  ++g_force_frame_count;
  board_led_toggle(BOARD_LED_NETWORK);
}

static void poll_uart(uint32_t now_ms) {
  bool restarted;
  uint16_t write_index;

  if (!board_force_uart_maintain_rx(dma_buffer, (uint16_t)sizeof(dma_buffer),
                                    &restarted)) {
    if (uart_rx_healthy) {
      ++g_force_io_error_count;
    }
    uart_rx_healthy = false;
    g_force_valid = 0U;
    return;
  }
  if (restarted) {
    force_sensor_parser_init(&parser);
    read_index = 0U;
    uart_rx_healthy = true;
    g_force_valid = 0U;
    ++g_force_uart_restart_count;
    active_since_ms = now_ms;
  }

  write_index = board_force_uart_rx_index((uint16_t)sizeof(dma_buffer));
  while (read_index != write_index) {
    ForceSensorFrame frame;
    ForceParseResult result =
        force_sensor_parser_feed(&parser, dma_buffer[read_index], &frame);
    read_index =
        (uint16_t)((read_index + 1U) % (uint16_t)sizeof(dma_buffer));
    if (result == FORCE_PARSE_ERROR) {
      ++g_force_parse_error_count;
    } else if (result == FORCE_PARSE_FRAME) {
      ForceSensorSample sample;
      if (force_sensor_decode_sample_order(
              &frame, (ForceSensorFloatOrder)FORCE_TEST_FLOAT_ORDER, &sample)) {
        store_sample(&sample, now_ms);
      } else {
        ++g_force_non_sample_frame_count;
      }
    }
  }
}

static void poll_commands(uint32_t now_ms) {
  switch (setup_phase) {
    case SENSOR_WAIT_BEFORE_STOP:
      if (time_reached(now_ms, setup_deadline_ms)) {
        (void)send_simple_command(FORCE_CMD_STOP_STREAM);
        setup_phase = SENSOR_WAIT_BEFORE_MODE;
        setup_deadline_ms = now_ms + 100U;
      }
      break;
    case SENSOR_WAIT_BEFORE_MODE:
      if (time_reached(now_ms, setup_deadline_ms)) {
#if FORCE_TEST_CONTINUOUS_MODE
        (void)send_simple_command(FORCE_CMD_START_STREAM_1KHZ);
#else
        (void)send_simple_command(FORCE_CMD_READ_ONCE);
#endif
        setup_phase = SENSOR_ACTIVE;
        active_since_ms = now_ms;
        next_request_ms = now_ms + FORCE_TEST_REQUEST_PERIOD_MS;
      }
      break;
    case SENSOR_ACTIVE:
#if !FORCE_TEST_CONTINUOUS_MODE
      if (time_reached(now_ms, next_request_ms)) {
        (void)send_simple_command(FORCE_CMD_READ_ONCE);
        next_request_ms = now_ms + FORCE_TEST_REQUEST_PERIOD_MS;
      }
#else
      (void)now_ms;
#endif
      break;
    default:
      Error_Handler();
      break;
  }
}

static void update_fault_led(uint32_t now_ms) {
  if (setup_phase == SENSOR_ACTIVE &&
      ((g_force_valid == 0U &&
        (uint32_t)(now_ms - active_since_ms) >
            FORCE_TEST_STALE_TIMEOUT_MS) ||
       (g_force_valid != 0U &&
        (uint32_t)(now_ms - g_force_last_sample_ms) >
            FORCE_TEST_STALE_TIMEOUT_MS))) {
    g_force_stale = 1U;
  }
  standalone_set_fault(g_force_stale != 0U || !uart_rx_healthy ||
                       g_force_io_error_count != 0U);
}

int main(void) {
  standalone_runtime_init(BOARD_PERIPHERAL_FORCE_UART);
  if (FORCE_TEST_FLOAT_ORDER > 1U || FORCE_TEST_REQUEST_PERIOD_MS == 0U ||
      !board_force_uart_start_rx(dma_buffer,
                                 (uint16_t)sizeof(dma_buffer))) {
    Error_Handler();
  }
  force_sensor_parser_init(&parser);
  setup_phase = SENSOR_WAIT_BEFORE_STOP;
  setup_deadline_ms = 250U;

  while (1) {
    uint32_t now_ms = HAL_GetTick();
    poll_uart(now_ms);
    poll_commands(now_ms);
    update_fault_led(now_ms);
    standalone_heartbeat_poll(now_ms);
  }
}
