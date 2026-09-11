#include "arm_controller.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "app_config.h"

/** @brief 初始化机械臂状态、逆运动学参数和夹爪阈值。 */
void arm_controller_init(ArmController *controller) {
  if (controller == NULL) {
    return;
  }
  memset(controller, 0, sizeof(*controller));
  arm_kinematics_config_default(&controller->kinematics);
  controller->last_ik.status = ARM_IK_OK;
  controller->state = ARM_CONTROL_IDLE;
  controller->grip_force_threshold_n = APP_GRIPPER_FORCE_THRESHOLD_N;
}

/** @brief 求解目标点并通过回调下发 J1~J5 关节位置。 */
ArmIkStatus arm_controller_move_to(ArmController *controller,
                                   const ArmPoint *target, uint32_t now_ms,
                                   ArmSetJointFn set_joint, void *context) {
  float solution[ARM_JOINT_COUNT];
  ArmIkStatus status;

  if (controller == NULL || target == NULL || set_joint == NULL) {
    return ARM_IK_BAD_ARGUMENT;
  }
  status = arm_kinematics_inverse(&controller->kinematics, target,
                                  controller->commanded_rad, solution,
                                  &controller->last_ik);
  if (status != ARM_IK_OK) {
    controller->state = ARM_CONTROL_ERROR;
    return status;
  }

  for (uint8_t joint = 0U; joint < ARM_JOINT_COUNT; ++joint) {
    if (!set_joint(joint, solution[joint], context)) {
      controller->state = ARM_CONTROL_ERROR;
      return ARM_IK_NUMERIC_ERROR;
    }
  }
  memcpy(controller->commanded_rad, solution,
         sizeof(controller->commanded_rad));
  controller->target = *target;
  controller->last_command_ms = now_ms;
  controller->state = ARM_CONTROL_MOVING;
  return ARM_IK_OK;
}

/** @brief 下发 J6 夹爪位置和电流限制。 */
bool arm_controller_move_gripper(ArmController *controller,
                                 float position_rad,
                                 uint16_t current_limit_ma,
                                 uint32_t now_ms,
                                 ArmSetJointFn set_joint, void *context) {
  if (controller == NULL || set_joint == NULL || !isfinite(position_rad) ||
      position_rad < APP_GRIPPER_MIN_POSITION_RAD ||
      position_rad > APP_GRIPPER_MAX_POSITION_RAD ||
      current_limit_ma == 0U ||
      current_limit_ma > APP_GRIPPER_MAX_CURRENT_MA ||
      !set_joint(5U, position_rad, context)) {
    if (controller != NULL) {
      controller->state = ARM_CONTROL_ERROR;
    }
    return false;
  }

  controller->gripper_target_rad = position_rad;
  controller->gripper_current_limit_ma = current_limit_ma;
  controller->last_command_ms = now_ms;
  controller->last_grip_force_n = 0.0F;
  controller->state = position_rad >= APP_GRIPPER_CLOSE_DIRECTION_THRESHOLD_RAD
                          ? ARM_CONTROL_GRIPPER_CLOSING
                          : ARM_CONTROL_GRIPPER_OPENING;
  return true;
}

/** @brief 根据六维力反馈判断夹持是否达到阈值。 */
void arm_controller_poll(ArmController *controller,
                         const ForceSensorSample *force,
                         bool force_is_valid,
                         ArmStopJointFn stop_joint, void *context) {
  float magnitude;

  if (controller == NULL || force == NULL || !force_is_valid ||
      controller->state != ARM_CONTROL_GRIPPER_CLOSING) {
    return;
  }
  magnitude = sqrtf(force->fx * force->fx + force->fy * force->fy +
                    force->fz * force->fz);
  if (!isfinite(magnitude)) {
    return;
  }
  controller->last_grip_force_n = magnitude;
  if (magnitude >= controller->grip_force_threshold_n &&
      stop_joint != NULL && stop_joint(5U, context)) {
    controller->state = ARM_CONTROL_GRIPPED;
  }
}

/** @brief 停止已跟踪的六个关节并设置停止状态。 */
void arm_controller_stop(ArmController *controller,
                         ArmStopJointFn stop_joint, void *context) {
  if (controller == NULL) {
    return;
  }
  if (stop_joint != NULL) {
    for (uint8_t joint = 0U; joint < ARM_MOTOR_COUNT; ++joint) {
      (void)stop_joint(joint, context);
    }
  }
  controller->state = ARM_CONTROL_STOPPED;
}
