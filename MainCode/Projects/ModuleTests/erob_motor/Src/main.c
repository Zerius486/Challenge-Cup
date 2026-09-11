#include <stdbool.h>
#include <stdint.h>

#include "app_bridge.h"
#include "board.h"
#include "byte_codec.h"
#include "erob_canopen.h"
#include "main.h"
#include "standalone_runtime.h"
#include "test_config.h"

volatile uint32_t g_erob_tx_count;
volatile uint32_t g_erob_rx_count;
volatile uint32_t g_erob_last_rx_ms;
volatile uint32_t g_erob_last_status_ms;
volatile uint32_t g_erob_can_error;
volatile uint32_t g_erob_last_abort_code;
volatile uint16_t g_erob_statusword;
volatile uint8_t g_erob_ds402_state;
volatile uint8_t g_erob_status_valid;
volatile uint8_t g_erob_test_failed;
volatile uint8_t g_erob_motion_phase;

typedef enum {
  MOTION_WAIT_FEEDBACK = 0,
  MOTION_NMT_START,
  MOTION_FAULT_RESET,
  MOTION_SHUTDOWN,
  MOTION_SWITCH_ON,
  MOTION_ENABLE,
  MOTION_SETTLE,
  MOTION_SET_MODE,
  MOTION_SET_PROFILE_VELOCITY,
  MOTION_SET_TARGET,
  MOTION_TRIGGER_LOW,
  MOTION_TRIGGER_HIGH,
  MOTION_HOLD,
  MOTION_STOP,
  MOTION_DONE
} MotionPhase;

static uint32_t last_status_request_ms;
#if EROB_TEST_ENABLE_MOTION
static uint32_t phase_deadline_ms;

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms) {
  return (int32_t)(now_ms - deadline_ms) >= 0;
}
#endif

static bool transmit(const CanFrame *frame) {
  if (!board_can_send(EROB_TEST_CAN_BUS, frame, 3U)) {
    g_erob_test_failed = 1U;
    standalone_set_fault(true);
    return false;
  }
  ++g_erob_tx_count;
  return true;
}

static void poll_status(uint32_t now_ms) {
  CanFrame frame;
  if ((uint32_t)(now_ms - last_status_request_ms) <
      EROB_TEST_STATUS_PERIOD_MS) {
    return;
  }
  last_status_request_ms = now_ms;
  if (!erob_make_statusword_read(EROB_TEST_NODE_ID, &frame) ||
      !transmit(&frame)) {
    g_erob_test_failed = 1U;
  }
}

#if EROB_TEST_ENABLE_MOTION
static bool send_controlword(uint16_t value) {
  CanFrame frame;
  return erob_make_controlword(EROB_TEST_NODE_ID, value, &frame) &&
         transmit(&frame);
}

static void next_phase(MotionPhase phase, uint32_t now_ms,
                       uint32_t delay_ms) {
  g_erob_motion_phase = (uint8_t)phase;
  phase_deadline_ms = now_ms + delay_ms;
}

static void motion_poll(uint32_t now_ms) {
  CanFrame frame;
  MotionPhase phase = (MotionPhase)g_erob_motion_phase;

  if (g_erob_test_failed != 0U && phase != MOTION_STOP &&
      phase != MOTION_DONE) {
    next_phase(MOTION_STOP, now_ms, 0U);
    phase = MOTION_STOP;
  }
  if (!time_reached(now_ms, phase_deadline_ms)) {
    return;
  }

  switch (phase) {
    case MOTION_WAIT_FEEDBACK:
      if (g_erob_status_valid == 0U || now_ms < 1000U) {
        return;
      }
      next_phase(MOTION_NMT_START, now_ms, 0U);
      break;
    case MOTION_NMT_START:
      if (erob_make_nmt(EROB_NMT_START, EROB_TEST_NODE_ID, &frame) &&
          transmit(&frame)) {
        next_phase(g_erob_ds402_state == EROB_DS402_FAULT
                       ? MOTION_FAULT_RESET
                       : MOTION_SHUTDOWN,
                   now_ms, EROB_TEST_COMMAND_STEP_MS);
      }
      break;
    case MOTION_FAULT_RESET:
      if (send_controlword(0x0080U)) {
        next_phase(MOTION_SHUTDOWN, now_ms, 200U);
      }
      break;
    case MOTION_SHUTDOWN:
      if (send_controlword(0x0006U)) {
        next_phase(MOTION_SET_MODE, now_ms, EROB_TEST_COMMAND_STEP_MS);
      }
      break;
    case MOTION_SWITCH_ON:
      if (send_controlword(0x0007U)) {
        next_phase(MOTION_ENABLE, now_ms, EROB_TEST_COMMAND_STEP_MS);
      }
      break;
    case MOTION_ENABLE:
      if (send_controlword(0x000FU)) {
        next_phase(MOTION_SETTLE, now_ms, EROB_TEST_ENABLE_SETTLE_MS);
      }
      break;
    case MOTION_SETTLE:
      if (g_erob_status_valid == 0U ||
          g_erob_ds402_state != EROB_DS402_OPERATION_ENABLED ||
          (uint32_t)(now_ms - g_erob_last_status_ms) > 250U) {
        g_erob_test_failed = 1U;
        standalone_set_fault(true);
        next_phase(MOTION_STOP, now_ms, 0U);
      } else {
        next_phase(MOTION_TRIGGER_LOW, now_ms, 0U);
      }
      break;
    case MOTION_SET_MODE:
      if (erob_make_mode(EROB_TEST_NODE_ID, EROB_MODE_PROFILE_POSITION,
                         &frame) &&
          transmit(&frame)) {
        next_phase(MOTION_SET_PROFILE_VELOCITY, now_ms,
                   EROB_TEST_COMMAND_STEP_MS);
      }
      break;
    case MOTION_SET_PROFILE_VELOCITY:
      if (erob_make_profile_velocity(
              EROB_TEST_NODE_ID, EROB_TEST_PROFILE_VELOCITY_PLUS_S, &frame) &&
          transmit(&frame)) {
        next_phase(MOTION_SET_TARGET, now_ms, EROB_TEST_COMMAND_STEP_MS);
      }
      break;
    case MOTION_SET_TARGET:
      if (erob_make_target_position(EROB_TEST_NODE_ID,
                                    EROB_TEST_TARGET_POSITION_PLUS, &frame) &&
          transmit(&frame)) {
        next_phase(MOTION_SWITCH_ON, now_ms, EROB_TEST_COMMAND_STEP_MS);
      }
      break;
    case MOTION_TRIGGER_LOW:
      if (send_controlword(0x000FU)) {
        next_phase(MOTION_TRIGGER_HIGH, now_ms, EROB_TEST_COMMAND_STEP_MS);
      }
      break;
    case MOTION_TRIGGER_HIGH:
      if (send_controlword(0x001FU)) {
        next_phase(MOTION_HOLD, now_ms, EROB_TEST_HOLD_MS);
      }
      break;
    case MOTION_HOLD:
      next_phase(MOTION_STOP, now_ms, 0U);
      break;
    case MOTION_STOP:
      (void)send_controlword(0x0002U);
      next_phase(MOTION_DONE, now_ms, 0U);
      break;
    case MOTION_DONE:
    default:
      break;
  }
}
#endif

void app_bridge_on_can_frame(uint8_t bus, const CanFrame *frame,
                             uint32_t now_ms) {
  ErobSdoResult result;
  if (bus != EROB_TEST_CAN_BUS ||
      !erob_parse_sdo_response(frame, &result) ||
      result.node_id != EROB_TEST_NODE_ID) {
    return;
  }

  ++g_erob_rx_count;
  g_erob_last_rx_ms = now_ms;
  board_led_toggle(BOARD_LED_NETWORK);
  if (result.kind == EROB_SDO_ABORT) {
    g_erob_last_abort_code = result.abort_code;
    g_erob_test_failed = 1U;
    standalone_set_fault(true);
  } else if (result.kind == EROB_SDO_UPLOAD_OK && result.index == 0x6041U &&
             result.value_size >= 2U) {
    g_erob_statusword = (uint16_t)result.value;
    g_erob_ds402_state = (uint8_t)erob_ds402_state(g_erob_statusword);
    g_erob_last_status_ms = now_ms;
    g_erob_status_valid = 1U;
  }
}

void app_bridge_on_can_error(uint8_t bus, uint32_t error) {
  if (bus == EROB_TEST_CAN_BUS) {
    g_erob_can_error = error;
    g_erob_test_failed = 1U;
    standalone_set_fault(true);
  }
}

int main(void) {
  standalone_runtime_init(BOARD_PERIPHERAL_CAN1);
  if (EROB_TEST_NODE_ID < EROB_NODE_ID_MIN ||
      EROB_TEST_NODE_ID > EROB_NODE_ID_MAX) {
    Error_Handler();
  }

#if EROB_TEST_ENABLE_MOTION
  g_erob_motion_phase = (uint8_t)MOTION_WAIT_FEEDBACK;
#else
  g_erob_motion_phase = (uint8_t)MOTION_DONE;
#endif

  while (1) {
    uint32_t now_ms = HAL_GetTick();
    poll_status(now_ms);
#if EROB_TEST_ENABLE_MOTION
    motion_poll(now_ms);
#endif
    standalone_heartbeat_poll(now_ms);
  }
}
