#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "arm_kinematics.h"
#include "force_sensor.h"

#define ARM_MOTOR_COUNT 6U

typedef enum {
  ARM_CONTROL_IDLE = 0,
  ARM_CONTROL_WAITING_FOR_MOTORS,
  ARM_CONTROL_MOVING,
  ARM_CONTROL_GRIPPER_OPENING,
  ARM_CONTROL_GRIPPER_CLOSING,
  ARM_CONTROL_GRIPPED,
  ARM_CONTROL_STOPPED,
  ARM_CONTROL_ERROR
} ArmControlState;

typedef bool (*ArmSetJointFn)(uint8_t joint_index, float position_rad,
                              void *context);
typedef bool (*ArmStopJointFn)(uint8_t joint_index, void *context);

typedef struct {
  ArmKinematicsConfig kinematics;
  float commanded_rad[ARM_JOINT_COUNT];
  ArmPoint target;
  ArmIkResult last_ik;
  ArmControlState state;
  float gripper_target_rad;
  uint16_t gripper_current_limit_ma;
  float grip_force_threshold_n;
  float last_grip_force_n;
  uint32_t last_command_ms;
} ArmController;

void arm_controller_init(ArmController *controller);
ArmIkStatus arm_controller_move_to(ArmController *controller,
                                   const ArmPoint *target, uint32_t now_ms,
                                   ArmSetJointFn set_joint, void *context);
bool arm_controller_move_gripper(ArmController *controller,
                                 float position_rad,
                                 uint16_t current_limit_ma,
                                 uint32_t now_ms,
                                 ArmSetJointFn set_joint, void *context);
void arm_controller_poll(ArmController *controller,
                         const ForceSensorSample *force,
                         bool force_is_valid,
                         ArmStopJointFn stop_joint, void *context);
void arm_controller_stop(ArmController *controller,
                         ArmStopJointFn stop_joint, void *context);

#endif
