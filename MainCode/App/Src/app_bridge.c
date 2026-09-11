#include "app_bridge.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "app_config.h"
#include "arm_controller.h"
#include "arm_nuc_protocol.h"
#include "board.h"
#include "byte_codec.h"
#include "erob_canopen.h"
#include "force_sensor.h"
#include "legacy_udp.h"
#include "robstride.h"
#include "udp_protocol.h"

enum {
  APP_REPLY_OK = 0,
  APP_REPLY_BAD_FRAME = 1,
  APP_REPLY_BAD_COMMAND = 2,
  APP_REPLY_SAFETY_LOCKED = 3,
  APP_REPLY_UNSUPPORTED = 4,
  APP_REPLY_IO_ERROR = 5,
  APP_REPLY_REPLAY = 6,
  APP_REPLY_NOT_READY = 7,
  APP_REPLY_UNREACHABLE = 8
};

typedef struct {
  bool used;
  uint8_t bus;
  uint8_t driver;
  uint8_t node_id;
  uint8_t model;
  bool erob_enable_commanded;
  bool erob_status_valid;
  uint16_t erob_statusword;
  uint32_t erob_status_timestamp_ms;
  uint32_t erob_ready_after_ms;
  bool robstride_enable_commanded;
  bool robstride_feedback_valid;
  uint8_t robstride_mode_state;
  uint8_t robstride_fault_bits;
  uint32_t robstride_feedback_timestamp_ms;
  uint32_t deadline_ms;
} ActiveMotor;

typedef struct {
  ForceSensorSample force;
  uint32_t force_timestamp_ms;
  bool force_valid;
  uint8_t erob_node;
  uint16_t erob_statusword;
  uint32_t erob_timestamp_ms;
  bool erob_valid;
  RobStrideFeedback robstride;
  RobStrideModel robstride_model;
  uint32_t robstride_timestamp_ms;
  bool robstride_valid;
  bool force_uart_fault_seen;
  uint32_t can_error[2];
} BridgeTelemetry;

static ForceSensorParser force_parser;
static uint8_t force_dma_buffer[APP_FORCE_RX_DMA_SIZE];
static uint16_t force_read_index;
static BridgeTelemetry telemetry;
static ActiveMotor active_motors[APP_MAX_TRACKED_MOTORS];
static bool have_last_sequence;
static uint32_t last_sequence;
static uint32_t last_erob_poll_ms;
static uint8_t erob_poll_cursor;
static ArmController arm_controller;
static uint8_t arm_sink_status;
static bool pending_position;
static ArmNucCommand pending_position_command;
static uint32_t pending_position_retry_ms;
static bool pending_gripper;
static ArmNucCommand pending_gripper_command;

static const uint8_t arm_node_ids[ARM_MOTOR_COUNT] = {
    APP_ARM_J1_NODE_ID, APP_ARM_J2_NODE_ID, APP_ARM_J3_NODE_ID,
    APP_ARM_J4_NODE_ID, APP_ARM_J5_NODE_ID, APP_ARM_J6_NODE_ID};

static bool time_reached(uint32_t now, uint32_t deadline) {
  return (int32_t)(now - deadline) >= 0;
}

static bool send_can(uint8_t bus, const CanFrame *frame) {
  return board_can_send(bus, frame, 3U);
}

static bool send_robstride_parameter(uint8_t bus, const CanFrame *frame) {
  if (!send_can(bus, frame)) {
    return false;
  }
  board_delay_ms(APP_ROBSTRIDE_PARAMETER_GAP_MS);
  return true;
}

static ActiveMotor *find_motor(uint8_t driver, uint8_t node_id) {
  for (size_t i = 0U; i < APP_MAX_TRACKED_MOTORS; ++i) {
    if (active_motors[i].used && active_motors[i].driver == driver &&
        active_motors[i].node_id == node_id) {
      return &active_motors[i];
    }
  }
  return NULL;
}

static ActiveMotor *track_motor(const UdpMotorCommand *command,
                                uint32_t now_ms) {
  ActiveMotor *free_slot = NULL;
  for (size_t i = 0U; i < APP_MAX_TRACKED_MOTORS; ++i) {
    if (active_motors[i].used &&
        active_motors[i].driver == command->driver &&
        active_motors[i].node_id == command->node_id) {
      active_motors[i].deadline_ms = now_ms + command->timeout_ms;
      active_motors[i].bus = command->can_bus;
      active_motors[i].model = (uint8_t)(command->options & 0x07U);
      return &active_motors[i];
    }
    if (!active_motors[i].used && free_slot == NULL) {
      free_slot = &active_motors[i];
    }
  }
  if (free_slot != NULL) {
    free_slot->used = true;
    free_slot->bus = command->can_bus;
    free_slot->driver = command->driver;
    free_slot->node_id = command->node_id;
    free_slot->model = (uint8_t)(command->options & 0x07U);
    free_slot->deadline_ms = now_ms + command->timeout_ms;
  }
  return free_slot;
}

static bool stop_motor(ActiveMotor *motor, uint32_t now_ms) {
  CanFrame frame;
  bool ok;
  pending_position = false;
  pending_gripper = false;
  if (motor->driver == UDP_DRIVER_EROB) {
    ok = erob_make_controlword(motor->node_id, 0x0002U, &frame) &&
         send_can(motor->bus, &frame);
  } else {
    ok = robstride_make_stop(motor->node_id, APP_ROBSTRIDE_MASTER_ID,
                             false, &frame) &&
         send_can(motor->bus, &frame);
  }
  if (ok) {
    memset(motor, 0, sizeof(*motor));
  } else {
    motor->deadline_ms = now_ms + APP_STOP_RETRY_PERIOD_MS;
  }
  return ok;
}

static bool stop_all(uint32_t now_ms) {
  bool ok = true;
  pending_position = false;
  pending_gripper = false;
  for (size_t i = 0U; i < APP_MAX_TRACKED_MOTORS; ++i) {
    if (active_motors[i].used &&
        !stop_motor(&active_motors[i], now_ms)) {
      ok = false;
    }
  }
  return ok;
}

static bool motion_is_unlocked(const UdpMotorCommand *command) {
#if APP_REMOTE_MOTION_ALLOWED
  return (command->options & APP_COMMAND_ARM_OPTION) != 0U;
#else
  (void)command;
  return false;
#endif
}

static bool erob_motion_ready(const ActiveMotor *motor, uint32_t now_ms) {
  bool ready;
  uint32_t critical_state = board_critical_enter();
  ready = motor->erob_enable_commanded &&
          time_reached(now_ms, motor->erob_ready_after_ms) &&
          motor->erob_status_valid &&
          erob_ds402_state(motor->erob_statusword) ==
              EROB_DS402_OPERATION_ENABLED &&
          (uint32_t)(now_ms - motor->erob_status_timestamp_ms) <=
              APP_EROB_STATUS_STALE_MS;
  board_critical_exit(critical_state);
  return ready;
}

static bool robstride_motion_ready(const ActiveMotor *motor,
                                   uint32_t now_ms) {
  bool ready;
  uint32_t critical_state = board_critical_enter();
  ready = motor->robstride_enable_commanded &&
          motor->robstride_feedback_valid &&
          motor->robstride_mode_state == 2U &&
          motor->robstride_fault_bits == 0U &&
          (uint32_t)(now_ms - motor->robstride_feedback_timestamp_ms) <=
              APP_ROBSTRIDE_FEEDBACK_STALE_MS;
  board_critical_exit(critical_state);
  return ready;
}

static bool send_erob_command(const UdpMotorCommand *command) {
  CanFrame frame;
  int32_t target;

  switch ((UdpMotorOperation)command->operation) {
    case UDP_MOTOR_DISABLE:
      return erob_make_controlword(command->node_id, 0x0000U, &frame) &&
             send_can(command->can_bus, &frame);
    case UDP_MOTOR_STOP:
      return erob_make_controlword(command->node_id, 0x0002U, &frame) &&
             send_can(command->can_bus, &frame);
    case UDP_MOTOR_FAULT_RESET:
      return erob_make_controlword(command->node_id, 0x0080U, &frame) &&
             send_can(command->can_bus, &frame);
    case UDP_MOTOR_ENABLE:
      return erob_make_controlword(command->node_id, 0x0006U, &frame) &&
             send_can(command->can_bus, &frame) &&
             erob_make_mode(command->node_id, EROB_MODE_PROFILE_VELOCITY,
                            &frame) &&
             send_can(command->can_bus, &frame) &&
             erob_make_target_velocity(command->node_id, 0, &frame) &&
             send_can(command->can_bus, &frame) &&
             erob_make_controlword(command->node_id, 0x0007U, &frame) &&
             send_can(command->can_bus, &frame) &&
             erob_make_controlword(command->node_id, 0x000FU, &frame) &&
             send_can(command->can_bus, &frame);
    case UDP_MOTOR_POSITION:
      if (command->position < -2147483648.0F ||
          command->position >= 2147483648.0F || command->velocity <= 0.0F ||
          command->velocity >= 4294967296.0F) {
        return false;
      }
      target = (int32_t)command->position;
      return erob_make_profile_velocity(command->node_id,
                                        (uint32_t)command->velocity, &frame) &&
             send_can(command->can_bus, &frame) &&
             erob_make_target_position(command->node_id, target, &frame) &&
             send_can(command->can_bus, &frame) &&
             erob_make_mode(command->node_id, EROB_MODE_PROFILE_POSITION,
                            &frame) &&
             send_can(command->can_bus, &frame) &&
             erob_make_controlword(command->node_id, 0x000FU, &frame) &&
             send_can(command->can_bus, &frame) &&
             erob_make_controlword(command->node_id, 0x001FU, &frame) &&
             send_can(command->can_bus, &frame);
    case UDP_MOTOR_VELOCITY:
      if (command->velocity < -2147483648.0F ||
          command->velocity >= 2147483648.0F) {
        return false;
      }
      target = (int32_t)command->velocity;
      return erob_make_target_velocity(command->node_id, target, &frame) &&
             send_can(command->can_bus, &frame) &&
             erob_make_mode(command->node_id, EROB_MODE_PROFILE_VELOCITY,
                            &frame) &&
             send_can(command->can_bus, &frame);
    case UDP_MOTOR_TORQUE:
      if (command->torque < -1000.0F || command->torque > 1000.0F) {
        return false;
      }
      return erob_make_target_torque(command->node_id,
                                     (int16_t)command->torque, &frame) &&
             send_can(command->can_bus, &frame) &&
             erob_make_mode(command->node_id, EROB_MODE_PROFILE_TORQUE,
                            &frame) &&
             send_can(command->can_bus, &frame);
    case UDP_MOTOR_SET_ZERO:
    case UDP_MOTOR_IMPEDANCE:
    default:
      return false;
  }
}

static bool send_robstride_command(const UdpMotorCommand *command) {
  CanFrame frame;
  RobStrideModel model = (RobStrideModel)(command->options & 0x07U);
  const RobStrideLimits *limits = robstride_limits(model);
  RobStrideMotionCommand motion;

  if (limits == NULL) {
    return false;
  }
  switch ((UdpMotorOperation)command->operation) {
    case UDP_MOTOR_DISABLE:
    case UDP_MOTOR_STOP:
      return robstride_make_stop(command->node_id, APP_ROBSTRIDE_MASTER_ID,
                                 false, &frame) &&
             send_can(command->can_bus, &frame);
    case UDP_MOTOR_FAULT_RESET:
      return robstride_make_stop(command->node_id, APP_ROBSTRIDE_MASTER_ID,
                                 true, &frame) &&
             send_can(command->can_bus, &frame);
    case UDP_MOTOR_ENABLE:
      return robstride_make_stop(command->node_id, APP_ROBSTRIDE_MASTER_ID,
                                 false, &frame) &&
             send_can(command->can_bus, &frame) &&
             robstride_make_write_parameter_u8(
                 command->node_id, APP_ROBSTRIDE_MASTER_ID,
                 ROBSTRIDE_PARAM_RUN_MODE, ROBSTRIDE_RUN_CURRENT, &frame) &&
             send_robstride_parameter(command->can_bus, &frame) &&
             robstride_make_write_parameter_f32(
                 command->node_id, APP_ROBSTRIDE_MASTER_ID,
                 ROBSTRIDE_PARAM_IQ_REF, 0.0F, &frame) &&
             send_robstride_parameter(command->can_bus, &frame) &&
             robstride_make_enable(command->node_id, APP_ROBSTRIDE_MASTER_ID,
                                   &frame) &&
             send_can(command->can_bus, &frame);
    case UDP_MOTOR_SET_ZERO:
      return robstride_make_stop(command->node_id, APP_ROBSTRIDE_MASTER_ID,
                                 false, &frame) &&
             send_can(command->can_bus, &frame) &&
             robstride_make_set_zero(command->node_id,
                                     APP_ROBSTRIDE_MASTER_ID, &frame) &&
             send_can(command->can_bus, &frame);
    case UDP_MOTOR_POSITION:
      if (command->position < -ROBSTRIDE_POSITION_LIMIT_RAD ||
          command->position > ROBSTRIDE_POSITION_LIMIT_RAD ||
          command->velocity <= 0.0F ||
          command->velocity > limits->velocity_rad_s) {
        return false;
      }
      return robstride_make_write_parameter_f32(
                 command->node_id, APP_ROBSTRIDE_MASTER_ID,
                 ROBSTRIDE_PARAM_PROFILE_SPEED, fabsf(command->velocity),
                 &frame) &&
             send_robstride_parameter(command->can_bus, &frame) &&
             robstride_make_write_parameter_f32(
                 command->node_id, APP_ROBSTRIDE_MASTER_ID,
                 ROBSTRIDE_PARAM_POSITION_REF, command->position, &frame) &&
             send_robstride_parameter(command->can_bus, &frame) &&
             robstride_make_write_parameter_u8(
                 command->node_id, APP_ROBSTRIDE_MASTER_ID,
                 ROBSTRIDE_PARAM_RUN_MODE, ROBSTRIDE_RUN_POSITION, &frame) &&
             send_robstride_parameter(command->can_bus, &frame);
    case UDP_MOTOR_VELOCITY:
      if (command->velocity < -limits->velocity_rad_s ||
          command->velocity > limits->velocity_rad_s) {
        return false;
      }
      return robstride_make_write_parameter_f32(
                 command->node_id, APP_ROBSTRIDE_MASTER_ID,
                 ROBSTRIDE_PARAM_SPEED_REF, command->velocity, &frame) &&
             send_robstride_parameter(command->can_bus, &frame) &&
             robstride_make_write_parameter_u8(
                 command->node_id, APP_ROBSTRIDE_MASTER_ID,
                 ROBSTRIDE_PARAM_RUN_MODE, ROBSTRIDE_RUN_VELOCITY, &frame) &&
             send_robstride_parameter(command->can_bus, &frame);
    case UDP_MOTOR_TORQUE:
      if (!isfinite(command->torque) ||
          command->torque < -limits->current_a ||
          command->torque > limits->current_a) {
        return false;
      }
      return robstride_make_write_parameter_f32(
                 command->node_id, APP_ROBSTRIDE_MASTER_ID,
                 ROBSTRIDE_PARAM_IQ_REF, command->torque, &frame) &&
             send_robstride_parameter(command->can_bus, &frame) &&
             robstride_make_write_parameter_u8(
                 command->node_id, APP_ROBSTRIDE_MASTER_ID,
                 ROBSTRIDE_PARAM_RUN_MODE, ROBSTRIDE_RUN_CURRENT, &frame) &&
             send_robstride_parameter(command->can_bus, &frame);
    case UDP_MOTOR_IMPEDANCE:
      motion.position_rad = command->position;
      motion.velocity_rad_s = command->velocity;
      motion.torque_nm = command->torque;
      motion.kp = command->kp;
      motion.kd = command->kd;
      return robstride_make_write_parameter_u8(
                 command->node_id, APP_ROBSTRIDE_MASTER_ID,
                 ROBSTRIDE_PARAM_RUN_MODE, ROBSTRIDE_RUN_MOTION, &frame) &&
             send_robstride_parameter(command->can_bus, &frame) &&
             robstride_make_motion(model, command->node_id, &motion, &frame) &&
             send_can(command->can_bus, &frame);
    default:
      return false;
  }
}

static uint8_t handle_motor_command(const UdpMotorCommand *command,
                                    uint32_t now_ms) {
  bool hazardous = command->operation != UDP_MOTOR_DISABLE &&
                   command->operation != UDP_MOTOR_STOP &&
                   command->operation != UDP_MOTOR_FAULT_RESET;
  bool ok;
  ActiveMotor *tracked = NULL;
  ActiveMotor *existing;

  if ((command->driver == UDP_DRIVER_EROB &&
       (command->can_bus != APP_EROB_CAN_BUS || command->node_id == 0U ||
        command->node_id > 127U ||
        (command->options & 0x7FFFU) != 0U)) ||
      (command->driver == UDP_DRIVER_ROBSTRIDE &&
       (command->can_bus != APP_ROBSTRIDE_CAN_BUS ||
        command->node_id == 0U || command->node_id > 127U ||
        (command->options & 0x7FF8U) != 0U ||
        (command->options & 0x0007U) >= ROBSTRIDE_MODEL_COUNT))) {
    return APP_REPLY_BAD_COMMAND;
  }
  if (hazardous && !motion_is_unlocked(command)) {
    return APP_REPLY_SAFETY_LOCKED;
  }

  existing = find_motor(command->driver, command->node_id);
  if (existing != NULL && command->driver == UDP_DRIVER_ROBSTRIDE &&
      (command->operation == UDP_MOTOR_STOP ||
       command->operation == UDP_MOTOR_DISABLE) &&
      existing->model != (uint8_t)(command->options & 0x07U)) {
    return APP_REPLY_BAD_COMMAND;
  }

  if (hazardous) {
    tracked = track_motor(command, now_ms);
    if (tracked == NULL) {
      return APP_REPLY_IO_ERROR;
    }
    if (command->driver == UDP_DRIVER_EROB &&
        (command->operation == UDP_MOTOR_POSITION ||
         command->operation == UDP_MOTOR_VELOCITY ||
         command->operation == UDP_MOTOR_TORQUE) &&
        !erob_motion_ready(tracked, now_ms)) {
      (void)stop_motor(tracked, now_ms);
      return APP_REPLY_NOT_READY;
    }
    if (command->driver == UDP_DRIVER_ROBSTRIDE &&
        (command->operation == UDP_MOTOR_POSITION ||
         command->operation == UDP_MOTOR_VELOCITY ||
         command->operation == UDP_MOTOR_TORQUE ||
         command->operation == UDP_MOTOR_IMPEDANCE) &&
        !robstride_motion_ready(tracked, now_ms)) {
      (void)stop_motor(tracked, now_ms);
      return APP_REPLY_NOT_READY;
    }
  }

  if (tracked != NULL && command->operation == UDP_MOTOR_ENABLE) {
    uint32_t critical_state = board_critical_enter();
    if (command->driver == UDP_DRIVER_EROB) {
      tracked->erob_enable_commanded = false;
      tracked->erob_status_valid = false;
    } else {
      tracked->robstride_enable_commanded = false;
      tracked->robstride_feedback_valid = false;
    }
    board_critical_exit(critical_state);
  }

  ok = (command->driver == UDP_DRIVER_EROB)
           ? send_erob_command(command)
           : send_robstride_command(command);
  if (!ok) {
    if (tracked != NULL) {
      (void)stop_motor(tracked, now_ms);
    }
    return APP_REPLY_IO_ERROR;
  }
  if (tracked != NULL && command->operation == UDP_MOTOR_ENABLE) {
    uint32_t critical_state = board_critical_enter();
    if (command->driver == UDP_DRIVER_EROB) {
      tracked->erob_enable_commanded = true;
      tracked->erob_ready_after_ms = now_ms + APP_EROB_ENABLE_SETTLE_MS;
      tracked->deadline_ms =
          tracked->erob_ready_after_ms + command->timeout_ms;
    } else {
      tracked->robstride_enable_commanded = true;
    }
    board_critical_exit(critical_state);
  }
  if (!hazardous) {
    if (existing != NULL) {
      memset(existing, 0, sizeof(*existing));
    }
  }
  return APP_REPLY_OK;
}

static uint8_t arm_motor_model(uint8_t joint_index) {
  return joint_index == 4U ? APP_ROBSTRIDE_J5_MODEL
                           : APP_ROBSTRIDE_J6_MODEL;
}

static void make_arm_motor_command(uint8_t joint_index, uint8_t operation,
                                   uint32_t now_ms,
                                   UdpMotorCommand *command) {
  (void)now_ms;
  memset(command, 0, sizeof(*command));
  command->can_bus = joint_index < 4U ? APP_EROB_CAN_BUS
                                      : APP_ROBSTRIDE_CAN_BUS;
  command->driver = joint_index < 4U ? UDP_DRIVER_EROB
                                     : UDP_DRIVER_ROBSTRIDE;
  command->node_id = arm_node_ids[joint_index];
  command->operation = operation;
  command->timeout_ms = APP_ARM_COMMAND_TIMEOUT_MS;
  command->options = APP_COMMAND_ARM_OPTION;
  if (joint_index >= 4U) {
    command->options = (uint16_t)(command->options |
                                  arm_motor_model(joint_index));
  }
}

static uint8_t arm_prepare_motors(uint32_t now_ms) {
#if !APP_REMOTE_MOTION_ALLOWED
  (void)now_ms;
  return APP_REPLY_SAFETY_LOCKED;
#else
  bool all_ready = true;
  for (uint8_t joint = 0U; joint < ARM_MOTOR_COUNT; ++joint) {
    uint8_t driver = joint < 4U ? UDP_DRIVER_EROB : UDP_DRIVER_ROBSTRIDE;
    ActiveMotor *motor = find_motor(driver, arm_node_ids[joint]);
    if (motor == NULL ||
        (driver == UDP_DRIVER_EROB && !motor->erob_enable_commanded) ||
        (driver == UDP_DRIVER_ROBSTRIDE &&
         !motor->robstride_enable_commanded)) {
      UdpMotorCommand command;
      uint8_t status;
      make_arm_motor_command(joint, UDP_MOTOR_ENABLE, now_ms, &command);
      status = handle_motor_command(&command, now_ms);
      if (status != APP_REPLY_OK) {
        return status;
      }
      motor = find_motor(driver, arm_node_ids[joint]);
    }
    if (motor == NULL ||
        (driver == UDP_DRIVER_EROB &&
         !erob_motion_ready(motor, now_ms)) ||
        (driver == UDP_DRIVER_ROBSTRIDE &&
         !robstride_motion_ready(motor, now_ms))) {
      all_ready = false;
    }
  }
  return all_ready ? APP_REPLY_OK : APP_REPLY_NOT_READY;
#endif
}

static bool arm_set_joint(uint8_t joint_index, float position_rad,
                          void *context) {
  UdpMotorCommand command;
  uint32_t now_ms;

  if (context == NULL || joint_index >= ARM_MOTOR_COUNT ||
      !isfinite(position_rad)) {
    arm_sink_status = APP_REPLY_BAD_COMMAND;
    return false;
  }
  now_ms = *(const uint32_t *)context;
    make_arm_motor_command(joint_index, UDP_MOTOR_POSITION, now_ms, &command);
  if (joint_index < 4U) {
    float counts = APP_EROB_ENCODER_MIDPOINT +
                   APP_ARM_DIRECTION * position_rad *
                       (APP_EROB_ENCODER_COUNTS_PER_REV /
                        (2.0F * 3.14159265358979323846F));
    if (!isfinite(counts) || counts < 0.0F ||
        counts >= APP_EROB_ENCODER_COUNTS_PER_REV) {
      arm_sink_status = APP_REPLY_BAD_COMMAND;
      return false;
    }
    command.position = roundf(counts);
    command.velocity = APP_EROB_PROFILE_VELOCITY_COUNTS_S;
  } else {
    command.position = APP_ARM_DIRECTION * position_rad;
    command.velocity = APP_ROBSTRIDE_PROFILE_SPEED_RAD_S;
  }
  arm_sink_status = handle_motor_command(&command, now_ms);
  return arm_sink_status == APP_REPLY_OK;
}

static bool arm_stop_joint(uint8_t joint_index, void *context) {
  uint8_t driver;
  ActiveMotor *motor;
  uint32_t now_ms = context != NULL ? *(const uint32_t *)context : 0U;

  if (joint_index >= ARM_MOTOR_COUNT) {
    return false;
  }
  driver = joint_index < 4U ? UDP_DRIVER_EROB : UDP_DRIVER_ROBSTRIDE;
  motor = find_motor(driver, arm_node_ids[joint_index]);
  return motor == NULL || stop_motor(motor, now_ms);
}

static uint8_t handle_arm_position(const ArmNucCommand *command,
                                   uint32_t now_ms) {
  ArmPoint target;
  float preview[ARM_JOINT_COUNT];
  ArmIkResult preview_result;
  ArmIkStatus ik_status;

  target.x_mm = (float)command->x_centi_mm * 0.01F;
  target.y_mm = (float)command->y_centi_mm * 0.01F;
  target.z_mm = (float)command->z_centi_mm * 0.01F;
  ik_status = arm_kinematics_inverse(
      &arm_controller.kinematics, &target, arm_controller.commanded_rad,
      preview, &preview_result);
  if (ik_status == ARM_IK_UNREACHABLE) {
    arm_controller.last_ik = preview_result;
    arm_controller.state = ARM_CONTROL_ERROR;
    return APP_REPLY_UNREACHABLE;
  }
  if (ik_status != ARM_IK_OK) {
    arm_controller.last_ik = preview_result;
    arm_controller.state = ARM_CONTROL_ERROR;
    return APP_REPLY_BAD_COMMAND;
  }

  {
    uint8_t status = arm_prepare_motors(now_ms);
    if (status != APP_REPLY_OK) {
      if (status == APP_REPLY_NOT_READY) {
        pending_position_command = *command;
        pending_position = true;
        pending_position_retry_ms = now_ms + APP_STOP_RETRY_PERIOD_MS;
      }
      arm_controller.state = status == APP_REPLY_NOT_READY
                                 ? ARM_CONTROL_WAITING_FOR_MOTORS
                                 : ARM_CONTROL_ERROR;
      return status;
    }
  }
  arm_sink_status = APP_REPLY_OK;
  ik_status = arm_controller_move_to(&arm_controller, &target, now_ms,
                                     arm_set_joint, &now_ms);
  if (ik_status == ARM_IK_UNREACHABLE) {
    return APP_REPLY_UNREACHABLE;
  }
  if (ik_status != ARM_IK_OK) {
    (void)stop_all(now_ms);
    return arm_sink_status == APP_REPLY_OK ? APP_REPLY_IO_ERROR
                                           : arm_sink_status;
  }
  pending_position = false;
  return APP_REPLY_OK;
}

static uint8_t handle_arm_gripper(const ArmNucCommand *command,
                                  uint32_t now_ms) {
  CanFrame frame;
  const RobStrideLimits *limits =
      robstride_limits((RobStrideModel)APP_ROBSTRIDE_J6_MODEL);
  float current_a = (float)command->gripper_current_ma * 0.001F;
  float position_rad = (float)command->gripper_millirad * 0.001F;
  uint8_t status;

  /* Reject an invalid target before it can enter the wait queue. */
  if (limits == NULL || current_a <= 0.0F || current_a > limits->current_a ||
      position_rad < APP_GRIPPER_MIN_POSITION_RAD ||
      position_rad > APP_GRIPPER_MAX_POSITION_RAD) {
    return APP_REPLY_BAD_COMMAND;
  }
  status = arm_prepare_motors(now_ms);

  if (status != APP_REPLY_OK) {
    if (status == APP_REPLY_NOT_READY) {
      pending_gripper_command = *command;
      pending_gripper = true;
    }
    arm_controller.state = status == APP_REPLY_NOT_READY
                               ? ARM_CONTROL_WAITING_FOR_MOTORS
                               : ARM_CONTROL_ERROR;
    return status;
  }
  if (!robstride_make_write_parameter_f32(
          APP_ARM_J6_NODE_ID, APP_ROBSTRIDE_MASTER_ID,
          ROBSTRIDE_PARAM_CURRENT_LIMIT, current_a, &frame) ||
      !send_robstride_parameter(APP_ROBSTRIDE_CAN_BUS, &frame)) {
    return APP_REPLY_BAD_COMMAND;
  }
  arm_sink_status = APP_REPLY_OK;
  if (!arm_controller_move_gripper(
          &arm_controller, position_rad,
          command->gripper_current_ma, now_ms, arm_set_joint, &now_ms)) {
    return arm_sink_status == APP_REPLY_OK ? APP_REPLY_IO_ERROR
                                           : arm_sink_status;
  }
  pending_gripper = false;
  return APP_REPLY_OK;
}

/** @brief 电机尚未就绪时重试保存的位置目标。 */
static void poll_pending_position(uint32_t now_ms) {
  uint8_t status;

  if (!pending_position) {
    return;
  }
  if (!time_reached(now_ms, pending_position_retry_ms)) {
    return;
  }
  status = handle_arm_position(&pending_position_command, now_ms);
  if (status != APP_REPLY_NOT_READY) {
    pending_position = false;
    if (status != APP_REPLY_OK) {
      arm_controller.state = ARM_CONTROL_ERROR;
    }
  } else {
    pending_position_retry_ms = now_ms + APP_STOP_RETRY_PERIOD_MS;
  }
}

/** @brief 机械臂到位后重试保存的夹爪目标。 */
static void poll_pending_gripper(uint32_t now_ms) {
  uint8_t status;

  if (!pending_gripper ||
      (arm_controller.state != ARM_CONTROL_MOVING &&
       arm_controller.state != ARM_CONTROL_WAITING_FOR_MOTORS) ||
      (uint32_t)(now_ms - arm_controller.last_command_ms) <
          APP_ARM_POSITION_SETTLE_MS) {
    return;
  }
  status = handle_arm_gripper(&pending_gripper_command, now_ms);
  if (status == APP_REPLY_OK) {
    pending_gripper = false;
  } else if (status != APP_REPLY_NOT_READY) {
    pending_gripper = false;
    arm_controller.state = ARM_CONTROL_ERROR;
  }
}

static uint8_t handle_force_command(const UdpPacketView *packet) {
  uint8_t bytes[6];
  size_t length;

  if (packet->payload_length < 1U || packet->payload_length > 2U) {
    return APP_REPLY_BAD_COMMAND;
  }
  switch ((ForceSensorCommand)packet->payload[0]) {
    case FORCE_CMD_STOP_STREAM:
    case FORCE_CMD_START_STREAM_1KHZ:
    case FORCE_CMD_READ_ONCE:
    case FORCE_CMD_SET_BAUD:
    case FORCE_CMD_READ_SERIAL:
    case FORCE_CMD_INITIALIZE:
    case FORCE_CMD_READ_FIRMWARE:
    case FORCE_CMD_TARE:
    case FORCE_CMD_EXIT_DEBUG:
    case FORCE_CMD_ENTER_DEBUG:
    case FORCE_CMD_READ_KG:
    case FORCE_CMD_READ_RAW_VOLTAGE:
    case FORCE_CMD_READ_NEWTON:
      break;
    default:
      return APP_REPLY_BAD_COMMAND;
  }
  if (packet->payload[0] == FORCE_CMD_SET_BAUD) {
    if (packet->payload_length != 2U) {
      return APP_REPLY_BAD_COMMAND;
    }
    length = force_sensor_build_baud_command(
        (ForceSensorBaudCode)packet->payload[1], bytes);
  } else {
    if (packet->payload_length != 1U) {
      return APP_REPLY_BAD_COMMAND;
    }
    length = force_sensor_build_simple_command(
        (ForceSensorCommand)packet->payload[0], bytes);
  }
  if (length == 0U) {
    return APP_REPLY_BAD_COMMAND;
  }
  return board_force_uart_send(bytes, (uint16_t)length, 10U)
             ? APP_REPLY_OK
             : APP_REPLY_IO_ERROR;
}

static size_t make_reply(uint8_t type, uint32_t sequence, uint8_t request_type,
                         uint8_t status, uint16_t detail, uint8_t *response,
                         size_t capacity) {
  uint8_t payload[4];
  payload[0] = request_type;
  payload[1] = status;
  codec_write_u16_le(&payload[2], detail);
  return udp_protocol_encode(type, 0U, sequence, payload, sizeof(payload),
                             response, capacity);
}

static size_t make_arm_ascii_reply(uint8_t status, uint8_t *response,
                                   size_t capacity) {
  const char *text;
  size_t length;

  switch (status) {
    case APP_REPLY_OK:
      text = "ARM:OK/";
      break;
    case APP_REPLY_NOT_READY:
      text = "ARM:WAIT/";
      break;
    case APP_REPLY_SAFETY_LOCKED:
      text = "ARM:LOCKED/";
      break;
    case APP_REPLY_UNREACHABLE:
      text = "ARM:UNREACHABLE/";
      break;
    default:
      text = "ARM:ERROR/";
      break;
  }
  length = strlen(text);
  if (response == NULL || capacity < length) {
    return 0U;
  }
  memcpy(response, text, length);
  return length;
}

/** @brief 初始化电机跟踪、力传感器解析器和机械臂控制器。 */
void app_bridge_init(void) {
  uint8_t force_mode_command[5];
  uint8_t stream_command[5];
  size_t force_mode_length;
  size_t stream_length;

  memset(&telemetry, 0, sizeof(telemetry));
  memset(active_motors, 0, sizeof(active_motors));
  arm_controller_init(&arm_controller);
  arm_sink_status = APP_REPLY_OK;
  pending_position = false;
  memset(&pending_position_command, 0, sizeof(pending_position_command));
  pending_position_retry_ms = 0U;
  pending_gripper = false;
  memset(&pending_gripper_command, 0, sizeof(pending_gripper_command));
  force_sensor_parser_init(&force_parser);
  force_read_index = 0U;
  have_last_sequence = false;
  last_sequence = 0U;
  last_erob_poll_ms = 0U;
  erob_poll_cursor = 0U;
  if (!board_force_uart_start_rx(force_dma_buffer,
                                 (uint16_t)sizeof(force_dma_buffer))) {
    telemetry.force_uart_fault_seen = true;
    return;
  }
  force_mode_length = force_sensor_build_simple_command(
      FORCE_CMD_READ_NEWTON, force_mode_command);
  if (force_mode_length == 0U ||
      !board_force_uart_send(force_mode_command, (uint16_t)force_mode_length,
                             10U)) {
    telemetry.force_uart_fault_seen = true;
  }
  stream_length = force_sensor_build_simple_command(
      FORCE_CMD_START_STREAM_1KHZ, stream_command);
  if (stream_length == 0U ||
      !board_force_uart_send(stream_command, (uint16_t)stream_length, 10U)) {
    telemetry.force_uart_fault_seen = true;
  }
}

static void poll_erob_status(uint32_t now_ms) {
  CanFrame frame;
  uint32_t sent = 0U;

  if ((uint32_t)(now_ms - last_erob_poll_ms) <
      APP_EROB_STATUS_POLL_PERIOD_MS) {
    return;
  }
  last_erob_poll_ms = now_ms;
  for (size_t checked = 0U;
       checked < APP_MAX_TRACKED_MOTORS &&
       sent < APP_EROB_STATUS_POLL_BUDGET;
       ++checked) {
    ActiveMotor *motor = &active_motors[erob_poll_cursor];
    erob_poll_cursor =
        (uint8_t)((erob_poll_cursor + 1U) % APP_MAX_TRACKED_MOTORS);
    if (motor->used && motor->driver == UDP_DRIVER_EROB) {
      if (erob_make_statusword_read(motor->node_id, &frame)) {
        (void)send_can(motor->bus, &frame);
      }
      ++sent;
    }
  }
}

static void poll_force_dma(uint32_t now_ms) {
  bool restarted;
  if (!board_force_uart_maintain_rx(
          force_dma_buffer, (uint16_t)sizeof(force_dma_buffer), &restarted)) {
    telemetry.force_valid = false;
    telemetry.force_uart_fault_seen = true;
    return;
  }
  if (restarted) {
    force_sensor_parser_init(&force_parser);
    force_read_index = 0U;
    telemetry.force_valid = false;
    telemetry.force_uart_fault_seen = true;
  }

  uint16_t write_index =
      board_force_uart_rx_index((uint16_t)sizeof(force_dma_buffer));
  while (force_read_index != write_index) {
    ForceSensorFrame frame;
    ForceParseResult result = force_sensor_parser_feed(
        &force_parser, force_dma_buffer[force_read_index], &frame);
    force_read_index = (uint16_t)((force_read_index + 1U) %
                                  (uint16_t)sizeof(force_dma_buffer));
    if (result == FORCE_PARSE_FRAME) {
      ForceSensorSample sample;
      if (force_sensor_decode_sample_order(
              &frame, (ForceSensorFloatOrder)APP_FORCE_FLOAT_ORDER, &sample) &&
          isfinite(sample.fx) && isfinite(sample.fy) && isfinite(sample.fz) &&
          isfinite(sample.mx) && isfinite(sample.my) && isfinite(sample.mz)) {
        telemetry.force = sample;
        telemetry.force_timestamp_ms = now_ms;
        telemetry.force_valid = true;
      }
    }
  }
  if (telemetry.force_valid &&
      (uint32_t)(now_ms - telemetry.force_timestamp_ms) >
          APP_FORCE_STALE_TIMEOUT_MS) {
    telemetry.force_valid = false;
  }
}

/** @brief 主循环轮询力传感器、状态超时和夹爪闭环。 */
void app_bridge_poll(uint32_t now_ms) {
  poll_force_dma(now_ms);
  arm_controller_poll(&arm_controller, &telemetry.force,
                      telemetry.force_valid, arm_stop_joint, &now_ms);
  poll_pending_position(now_ms);
  poll_pending_gripper(now_ms);
  for (size_t i = 0U; i < APP_MAX_TRACKED_MOTORS; ++i) {
    if (active_motors[i].used &&
        time_reached(now_ms, active_motors[i].deadline_ms)) {
      (void)stop_motor(&active_motors[i], now_ms);
    }
  }
  poll_erob_status(now_ms);
}

void app_bridge_on_can_frame(uint8_t bus, const CanFrame *frame,
                             uint32_t now_ms) {
  ErobSdoResult sdo;
  RobStrideFeedback feedback;
  RobStrideParameterReply parameter;

  if (frame == NULL) {
    return;
  }
  if (bus == APP_EROB_CAN_BUS && erob_parse_sdo_response(frame, &sdo)) {
    if (sdo.kind == EROB_SDO_UPLOAD_OK && sdo.index == 0x6041U &&
        sdo.value_size >= 2U) {
      telemetry.erob_node = sdo.node_id;
      telemetry.erob_statusword = (uint16_t)sdo.value;
      telemetry.erob_timestamp_ms = now_ms;
      telemetry.erob_valid = true;
      ActiveMotor *motor = find_motor(UDP_DRIVER_EROB, sdo.node_id);
      if (motor != NULL) {
        motor->erob_statusword = (uint16_t)sdo.value;
        motor->erob_status_timestamp_ms = now_ms;
        motor->erob_status_valid = true;
      }
    }
    return;
  }
  if (bus == APP_ROBSTRIDE_CAN_BUS) {
    RobStrideModel model = ROBSTRIDE_RS01;
    uint8_t motor_id = (uint8_t)(frame->id >> 8);
    for (size_t i = 0U; i < APP_MAX_TRACKED_MOTORS; ++i) {
      if (active_motors[i].used &&
          active_motors[i].driver == UDP_DRIVER_ROBSTRIDE &&
          active_motors[i].node_id == motor_id &&
          active_motors[i].model < ROBSTRIDE_MODEL_COUNT) {
        model = (RobStrideModel)active_motors[i].model;
        break;
      }
    }
    if (robstride_parse_feedback(model, APP_ROBSTRIDE_MASTER_ID, frame,
                                 &feedback)) {
      ActiveMotor *motor;
      telemetry.robstride = feedback;
      telemetry.robstride_model = model;
      telemetry.robstride_timestamp_ms = now_ms;
      telemetry.robstride_valid = true;
      motor = find_motor(UDP_DRIVER_ROBSTRIDE, feedback.motor_id);
      if (motor != NULL) {
        motor->robstride_mode_state = feedback.mode_state;
        motor->robstride_fault_bits = feedback.fault_bits;
        motor->robstride_feedback_timestamp_ms = now_ms;
        motor->robstride_feedback_valid = true;
      }
    } else {
      (void)robstride_parse_parameter_reply(APP_ROBSTRIDE_MASTER_ID, frame,
                                             &parameter);
    }
  }
}

/** @brief 记录指定 CAN 总线最近一次 HAL 错误码。 */
void app_bridge_on_can_error(uint8_t bus, uint32_t error) {
  if (bus == 1U || bus == 2U) {
    telemetry.can_error[bus - 1U] = error;
  }
}

/** @brief 处理二进制 UDP、机械臂 ASCII 和兼容停机指令。 */
size_t app_bridge_handle_datagram(const uint8_t *datagram, size_t length,
                                  uint32_t now_ms, uint8_t *response,
                                  size_t response_capacity) {
  UdpPacketView packet;
  UdpDecodeStatus decoded = udp_protocol_decode(datagram, length, &packet);
  uint8_t status = APP_REPLY_OK;
  uint16_t detail = 0U;

  if (decoded != UDP_DECODE_OK) {
    ArmNucCommand arm_command;
    LegacyUdpCommand legacy;
    if (arm_nuc_parse(datagram, length, &arm_command)) {
      switch (arm_command.type) {
        case ARM_NUC_POSITION:
          pending_position = false;
          pending_gripper = false;
          status = handle_arm_position(&arm_command, now_ms);
          break;
        case ARM_NUC_GRIPPER:
          pending_position = false;
          pending_gripper = false;
          status = handle_arm_gripper(&arm_command, now_ms);
          break;
        case ARM_NUC_BUNDLE:
          pending_position = false;
          pending_gripper = false;
          status = arm_command.has_position
                       ? handle_arm_position(&arm_command, now_ms)
                       : APP_REPLY_OK;
          if ((status == APP_REPLY_OK || status == APP_REPLY_NOT_READY) &&
              arm_command.has_gripper) {
            pending_gripper_command = arm_command;
            pending_gripper = true;
          }
          break;
        case ARM_NUC_STOP:
          pending_position = false;
          pending_gripper = false;
          arm_controller_stop(&arm_controller, arm_stop_joint, &now_ms);
          /* arm_controller_stop() intentionally keeps the callback simple and
           * does not aggregate I/O failures; retry any remaining tracked
           * motor stop here and report the actual result to the NUC. */
          status = stop_all(now_ms) ? APP_REPLY_OK : APP_REPLY_IO_ERROR;
          break;
        default:
          status = APP_REPLY_BAD_COMMAND;
          break;
      }
      return make_arm_ascii_reply(status, response, response_capacity);
    }
    if (legacy_udp_parse(datagram, length, &legacy) &&
        legacy.stop_requested) {
      arm_controller_stop(&arm_controller, arm_stop_joint, &now_ms);
      status = stop_all(now_ms) ? APP_REPLY_OK : APP_REPLY_IO_ERROR;
      return make_reply(status == APP_REPLY_OK ? UDP_MSG_ACK : UDP_MSG_ERROR,
                        0U, UDP_MSG_SAFE_STOP, status, 0U, response,
                        response_capacity);
    }
    return make_reply(UDP_MSG_ERROR, 0U, 0U, APP_REPLY_BAD_FRAME,
                      (uint16_t)decoded, response, response_capacity);
  }

  if (packet.flags != 0U) {
    return make_reply(UDP_MSG_ERROR, packet.sequence, packet.type,
                      APP_REPLY_BAD_COMMAND, 0U, response,
                      response_capacity);
  }

  if (packet.type != UDP_MSG_SAFE_STOP && packet.type != UDP_MSG_HELLO &&
      have_last_sequence && (int32_t)(packet.sequence - last_sequence) <= 0) {
    return make_reply(UDP_MSG_ERROR, packet.sequence, packet.type,
                      APP_REPLY_REPLAY, 0U, response, response_capacity);
  }

  switch ((UdpMessageType)packet.type) {
    case UDP_MSG_HELLO:
      if (packet.payload_length != 0U) {
        status = APP_REPLY_BAD_COMMAND;
      } else {
        have_last_sequence = false;
      }
      break;
    case UDP_MSG_SAFE_STOP:
      status = packet.payload_length == 0U
                   ? (stop_all(now_ms) ? APP_REPLY_OK : APP_REPLY_IO_ERROR)
                   : APP_REPLY_BAD_COMMAND;
      break;
    case UDP_MSG_MOTOR_COMMAND: {
      UdpMotorCommand command;
      if (!udp_protocol_decode_motor_command(&packet, &command)) {
        status = APP_REPLY_BAD_COMMAND;
      } else {
        status = handle_motor_command(&command, now_ms);
      }
      break;
    }
    case UDP_MSG_FORCE_COMMAND:
      status = handle_force_command(&packet);
      break;
    default:
      status = APP_REPLY_UNSUPPORTED;
      break;
  }

  if (packet.type != UDP_MSG_HELLO && packet.type != UDP_MSG_SAFE_STOP &&
      status != APP_REPLY_REPLAY) {
    last_sequence = packet.sequence;
    have_last_sequence = true;
  }
  return make_reply(status == APP_REPLY_OK ? UDP_MSG_ACK : UDP_MSG_ERROR,
                    packet.sequence, packet.type, status, detail, response,
                    response_capacity);
}

/** @brief 编码力传感器和两类电机的统一遥测负载。 */
size_t app_bridge_build_telemetry(uint32_t now_ms, uint8_t *payload,
                                  size_t capacity) {
  BridgeTelemetry snapshot;
  uint32_t flags = 0U;
  uint32_t critical_state;
  if (payload == NULL || capacity < 68U) {
    return 0U;
  }
  critical_state = board_critical_enter();
  snapshot = telemetry;
  board_critical_exit(critical_state);
  memset(payload, 0, 68U);
  if (snapshot.force_valid) {
    flags |= 1U << 0;
  }
  if (snapshot.erob_valid) {
    flags |= 1U << 1;
  }
  if (snapshot.robstride_valid) {
    flags |= 1U << 2;
  }
  if (snapshot.force_uart_fault_seen) {
    flags |= 1U << 3;
  }
#if !APP_REMOTE_MOTION_ALLOWED
  flags |= 1U << 8;
#endif
  if (snapshot.can_error[0] != 0U) {
    flags |= 1U << 16;
  }
  if (snapshot.can_error[1] != 0U) {
    flags |= 1U << 17;
  }

  codec_write_u32_le(&payload[0], now_ms);
  codec_write_u32_le(&payload[4], flags);
  codec_write_u32_le(&payload[8], snapshot.force_timestamp_ms);
  codec_write_f32_le(&payload[12], snapshot.force.fx);
  codec_write_f32_le(&payload[16], snapshot.force.fy);
  codec_write_f32_le(&payload[20], snapshot.force.fz);
  codec_write_f32_le(&payload[24], snapshot.force.mx);
  codec_write_f32_le(&payload[28], snapshot.force.my);
  codec_write_f32_le(&payload[32], snapshot.force.mz);
  codec_write_u32_le(&payload[36], snapshot.erob_timestamp_ms);
  payload[40] = snapshot.erob_node;
  payload[41] = (uint8_t)erob_ds402_state(snapshot.erob_statusword);
  codec_write_u16_le(&payload[42], snapshot.erob_statusword);
  codec_write_u32_le(&payload[44], snapshot.robstride_timestamp_ms);
  payload[48] = snapshot.robstride.motor_id;
  payload[49] = snapshot.robstride.mode_state;
  payload[50] = snapshot.robstride.fault_bits;
  payload[51] = (uint8_t)snapshot.robstride_model;
  codec_write_f32_le(&payload[52], snapshot.robstride.position_rad);
  codec_write_f32_le(&payload[56], snapshot.robstride.velocity_rad_s);
  codec_write_f32_le(&payload[60], snapshot.robstride.torque_nm);
  codec_write_f32_le(&payload[64], snapshot.robstride.temperature_c);
  return 68U;
}

/** @brief 编码机械臂状态、目标和逆解误差负载。 */
size_t app_bridge_build_arm_status(uint32_t now_ms, uint8_t *payload,
                                   size_t capacity) {
  if (payload == NULL || capacity < 64U) {
    return 0U;
  }
  memset(payload, 0, 64U);
  codec_write_u32_le(&payload[0], now_ms);
  payload[4] = (uint8_t)arm_controller.state;
  payload[5] = (uint8_t)arm_controller.last_ik.status;
  codec_write_u16_le(&payload[6], arm_controller.gripper_current_limit_ma);
  codec_write_f32_le(&payload[8], arm_controller.target.x_mm);
  codec_write_f32_le(&payload[12], arm_controller.target.y_mm);
  codec_write_f32_le(&payload[16], arm_controller.target.z_mm);
  for (size_t i = 0U; i < ARM_JOINT_COUNT; ++i) {
    codec_write_f32_le(&payload[20U + i * 4U],
                       arm_controller.commanded_rad[i]);
  }
  codec_write_f32_le(&payload[40], arm_controller.gripper_target_rad);
  codec_write_f32_le(&payload[44], arm_controller.last_grip_force_n);
  codec_write_u32_le(&payload[48], arm_controller.last_command_ms);
  codec_write_f32_le(&payload[52], arm_controller.last_ik.position_error_mm);
  codec_write_f32_le(&payload[56], arm_controller.last_ik.orientation_error_rad);
  codec_write_f32_le(&payload[60],
                     arm_controller.grip_force_threshold_n);
  return 64U;
}
