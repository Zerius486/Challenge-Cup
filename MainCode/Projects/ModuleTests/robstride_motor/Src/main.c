#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "app_bridge.h"
#include "board.h"
#include "main.h"
#include "robstride.h"
#include "standalone_runtime.h"
#include "test_config.h"

volatile uint32_t g_robstride_tx_count;
volatile uint32_t g_robstride_rx_count;
volatile uint32_t g_robstride_last_rx_ms;
volatile uint32_t g_robstride_last_feedback_ms;
volatile uint32_t g_robstride_can_error;
volatile uint32_t g_robstride_faults;
volatile uint32_t g_robstride_warnings;
volatile uint16_t g_robstride_last_parameter_index;
volatile uint32_t g_robstride_last_parameter_raw;
volatile float g_robstride_last_parameter_value;
volatile float g_robstride_position_rad;
volatile float g_robstride_velocity_rad_s;
volatile float g_robstride_torque_nm;
volatile float g_robstride_temperature_c;
volatile float g_robstride_current_limit_a;
volatile uint8_t g_robstride_mode_state;
volatile uint8_t g_robstride_fault_bits;
volatile uint8_t g_robstride_parameter_success;
volatile uint8_t g_robstride_feedback_valid;
volatile uint8_t g_robstride_test_failed;
volatile uint8_t g_robstride_motion_phase;

typedef enum {
  MOTION_WAIT_FEEDBACK = 0,
  MOTION_CLEAR_FAULT,
  MOTION_SET_CURRENT_MODE,
  MOTION_ZERO_CURRENT,
  MOTION_ENABLE,
  MOTION_SETTLE,
  MOTION_APPLY_CURRENT,
  MOTION_HOLD,
  MOTION_ZERO_AFTER_TEST,
  MOTION_STOP,
  MOTION_DONE
} MotionPhase;

static const uint16_t diagnostic_parameters[] = {
    ROBSTRIDE_PARAM_POSITION,
    ROBSTRIDE_PARAM_SPEED,
    ROBSTRIDE_PARAM_IQ,
    ROBSTRIDE_PARAM_BUS_VOLTAGE,
};
static uint32_t last_diagnostic_ms;
static uint8_t diagnostic_cursor;
#if ROBSTRIDE_TEST_ENABLE_MOTION
static uint32_t phase_deadline_ms;

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms) {
  return (int32_t)(now_ms - deadline_ms) >= 0;
}
#endif

static bool transmit(const CanFrame *frame) {
  if (!board_can_send(ROBSTRIDE_TEST_CAN_BUS, frame, 3U)) {
    g_robstride_test_failed = 1U;
    standalone_set_fault(true);
    return false;
  }
  ++g_robstride_tx_count;
  return true;
}

static void poll_diagnostics(uint32_t now_ms) {
  CanFrame frame;
  uint16_t index;
  if ((uint32_t)(now_ms - last_diagnostic_ms) <
      ROBSTRIDE_TEST_DIAGNOSTIC_PERIOD_MS) {
    return;
  }
  last_diagnostic_ms = now_ms;
  index = diagnostic_parameters[diagnostic_cursor];
  diagnostic_cursor = (uint8_t)((diagnostic_cursor + 1U) %
                                (sizeof(diagnostic_parameters) /
                                 sizeof(diagnostic_parameters[0])));
  if (!robstride_make_read_parameter(ROBSTRIDE_TEST_MOTOR_ID,
                                     ROBSTRIDE_TEST_MASTER_ID, index,
                                     &frame) ||
      !transmit(&frame)) {
    g_robstride_test_failed = 1U;
  }
}

#if ROBSTRIDE_TEST_ENABLE_MOTION
static void next_phase(MotionPhase phase, uint32_t now_ms,
                       uint32_t delay_ms) {
  g_robstride_motion_phase = (uint8_t)phase;
  phase_deadline_ms = now_ms + delay_ms;
}

static bool write_current(float current_a) {
  CanFrame frame;
  return robstride_make_write_parameter_f32(
             ROBSTRIDE_TEST_MOTOR_ID, ROBSTRIDE_TEST_MASTER_ID,
             ROBSTRIDE_PARAM_IQ_REF, current_a, &frame) &&
         transmit(&frame);
}

static void motion_poll(uint32_t now_ms) {
  CanFrame frame;
  MotionPhase phase = (MotionPhase)g_robstride_motion_phase;

  if (g_robstride_test_failed != 0U && phase != MOTION_ZERO_AFTER_TEST &&
      phase != MOTION_STOP && phase != MOTION_DONE) {
    next_phase(MOTION_ZERO_AFTER_TEST, now_ms, 0U);
    phase = MOTION_ZERO_AFTER_TEST;
  }
  if (!time_reached(now_ms, phase_deadline_ms)) {
    return;
  }

  switch (phase) {
    case MOTION_WAIT_FEEDBACK:
      if (g_robstride_rx_count == 0U || now_ms < 1000U) {
        return;
      }
      next_phase(MOTION_CLEAR_FAULT, now_ms, 0U);
      break;
    case MOTION_CLEAR_FAULT:
      if (robstride_make_stop(ROBSTRIDE_TEST_MOTOR_ID,
                              ROBSTRIDE_TEST_MASTER_ID, true, &frame) &&
          transmit(&frame)) {
        next_phase(MOTION_SET_CURRENT_MODE, now_ms,
                   ROBSTRIDE_TEST_COMMAND_STEP_MS);
      }
      break;
    case MOTION_SET_CURRENT_MODE:
      if (robstride_make_write_parameter_u8(
              ROBSTRIDE_TEST_MOTOR_ID, ROBSTRIDE_TEST_MASTER_ID,
              ROBSTRIDE_PARAM_RUN_MODE, ROBSTRIDE_RUN_CURRENT, &frame) &&
          transmit(&frame)) {
        next_phase(MOTION_ZERO_CURRENT, now_ms,
                   ROBSTRIDE_TEST_COMMAND_STEP_MS);
      }
      break;
    case MOTION_ZERO_CURRENT:
      if (write_current(0.0F)) {
        next_phase(MOTION_ENABLE, now_ms, ROBSTRIDE_TEST_COMMAND_STEP_MS);
      }
      break;
    case MOTION_ENABLE:
      if (robstride_make_enable(ROBSTRIDE_TEST_MOTOR_ID,
                                ROBSTRIDE_TEST_MASTER_ID, &frame) &&
          transmit(&frame)) {
        next_phase(MOTION_SETTLE, now_ms, ROBSTRIDE_TEST_ENABLE_SETTLE_MS);
      }
      break;
    case MOTION_SETTLE:
      if (g_robstride_feedback_valid == 0U ||
          g_robstride_mode_state != 2U || g_robstride_fault_bits != 0U ||
          (uint32_t)(now_ms - g_robstride_last_feedback_ms) > 250U) {
        g_robstride_test_failed = 1U;
        standalone_set_fault(true);
        next_phase(MOTION_ZERO_AFTER_TEST, now_ms, 0U);
      } else {
        next_phase(MOTION_APPLY_CURRENT, now_ms, 0U);
      }
      break;
    case MOTION_APPLY_CURRENT:
      if (write_current(ROBSTRIDE_TEST_CURRENT_A)) {
        next_phase(MOTION_HOLD, now_ms, ROBSTRIDE_TEST_HOLD_MS);
      }
      break;
    case MOTION_HOLD:
      next_phase(MOTION_ZERO_AFTER_TEST, now_ms, 0U);
      break;
    case MOTION_ZERO_AFTER_TEST:
      (void)write_current(0.0F);
      next_phase(MOTION_STOP, now_ms, ROBSTRIDE_TEST_COMMAND_STEP_MS);
      break;
    case MOTION_STOP:
      if (robstride_make_stop(ROBSTRIDE_TEST_MOTOR_ID,
                              ROBSTRIDE_TEST_MASTER_ID, false, &frame)) {
        (void)transmit(&frame);
      }
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
  RobStrideFeedback feedback;
  RobStrideParameterReply parameter;
  RobStrideFaultReply fault;
  RobStrideModel model = (RobStrideModel)ROBSTRIDE_TEST_MODEL;

  if (bus != ROBSTRIDE_TEST_CAN_BUS) {
    return;
  }
  if (robstride_parse_feedback(model, ROBSTRIDE_TEST_MASTER_ID, frame,
                               &feedback) &&
      feedback.motor_id == ROBSTRIDE_TEST_MOTOR_ID) {
    g_robstride_position_rad = feedback.position_rad;
    g_robstride_velocity_rad_s = feedback.velocity_rad_s;
    g_robstride_torque_nm = feedback.torque_nm;
    g_robstride_temperature_c = feedback.temperature_c;
    g_robstride_mode_state = feedback.mode_state;
    g_robstride_fault_bits = feedback.fault_bits;
    g_robstride_last_feedback_ms = now_ms;
    g_robstride_feedback_valid = 1U;
  } else if (robstride_parse_parameter_reply(ROBSTRIDE_TEST_MASTER_ID, frame,
                                              &parameter) &&
             parameter.motor_id == ROBSTRIDE_TEST_MOTOR_ID) {
    g_robstride_last_parameter_index = parameter.index;
    g_robstride_last_parameter_raw = parameter.raw_value;
    g_robstride_last_parameter_value = parameter.float_value;
    g_robstride_parameter_success = parameter.success ? 1U : 0U;
    if (!parameter.success) {
      g_robstride_test_failed = 1U;
      standalone_set_fault(true);
    }
  } else if (robstride_parse_fault_reply(ROBSTRIDE_TEST_MASTER_ID, frame,
                                          &fault) &&
             fault.motor_id == ROBSTRIDE_TEST_MOTOR_ID) {
    g_robstride_faults = fault.faults;
    g_robstride_warnings = fault.warnings;
  } else {
    return;
  }

  ++g_robstride_rx_count;
  g_robstride_last_rx_ms = now_ms;
  board_led_toggle(BOARD_LED_NETWORK);
}

void app_bridge_on_can_error(uint8_t bus, uint32_t error) {
  if (bus == ROBSTRIDE_TEST_CAN_BUS) {
    g_robstride_can_error = error;
    g_robstride_test_failed = 1U;
    standalone_set_fault(true);
  }
}

int main(void) {
  const RobStrideLimits *limits;
  standalone_runtime_init(BOARD_PERIPHERAL_CAN2);
  limits = robstride_limits((RobStrideModel)ROBSTRIDE_TEST_MODEL);
  if (limits == NULL || ROBSTRIDE_TEST_MOTOR_ID == 0U ||
      ROBSTRIDE_TEST_MOTOR_ID > 127U) {
    Error_Handler();
  }
  g_robstride_current_limit_a = limits->current_a;

#if ROBSTRIDE_TEST_ENABLE_MOTION
  if (!isfinite(ROBSTRIDE_TEST_CURRENT_A) ||
      fabsf(ROBSTRIDE_TEST_CURRENT_A) > limits->current_a) {
    Error_Handler();
  }
  g_robstride_motion_phase = (uint8_t)MOTION_WAIT_FEEDBACK;
#else
  g_robstride_motion_phase = (uint8_t)MOTION_DONE;
#endif

  while (1) {
    uint32_t now_ms = HAL_GetTick();
    poll_diagnostics(now_ms);
#if ROBSTRIDE_TEST_ENABLE_MOTION
    motion_poll(now_ms);
#endif
    standalone_heartbeat_poll(now_ms);
  }
}
