#include "arm_kinematics.h"

#include <math.h>
#include <stddef.h>

#define ARM_EPSILON 1.0e-6F
#define ARM_MIN_DAMPING_MM 0.001F
#define ARM_MIN_STEP_RAD 0.001F
#define ARM_MAX_ITERATIONS 100U
#define ARM_MAX_LINE_SEARCH 6U
#define ARM_PI 3.14159265358979323846F
#define ARM_TWO_PI (2.0F * ARM_PI)

#define ARM_DEG_TO_RAD (ARM_PI / 180.0F)

typedef struct {
  float rho_mm;
  float z_mm;
  float tool_pitch_rad;
  float jacobian[2][ARM_LINK_COUNT];
} PlanarState;

static bool finite_float(float value) {
  return isfinite(value) != 0;
}

/** @brief 写入本机械臂长度、限位和水平末端默认参数。 */
void arm_kinematics_config_default(ArmKinematicsConfig *config) {
  static const float lengths[ARM_LINK_COUNT] = {
      419.9F, 122.95F, 124.0F, 208.12F};
  static const float minimum[ARM_JOINT_COUNT] = {
      -170.0F * ARM_DEG_TO_RAD,
      -90.0F * ARM_DEG_TO_RAD,
      -150.0F * ARM_DEG_TO_RAD,
      -150.0F * ARM_DEG_TO_RAD,
      -180.0F * ARM_DEG_TO_RAD};
  static const float maximum[ARM_JOINT_COUNT] = {
      170.0F * ARM_DEG_TO_RAD,
      90.0F * ARM_DEG_TO_RAD,
      150.0F * ARM_DEG_TO_RAD,
      150.0F * ARM_DEG_TO_RAD,
      180.0F * ARM_DEG_TO_RAD};

  if (config == NULL) {
    return;
  }
  *config = (ArmKinematicsConfig){0};
  for (size_t i = 0U; i < ARM_LINK_COUNT; ++i) {
    config->link_length_mm[i] = lengths[i];
  }
  for (size_t i = 0U; i < ARM_JOINT_COUNT; ++i) {
    config->joint_min_rad[i] = minimum[i];
    config->joint_max_rad[i] = maximum[i];
    config->preferred_rad[i] = 0.0F;
  }
  config->damping_mm = 5.0F;
  config->secondary_gain = 0.01F;
  config->max_step_rad = 0.20F;
  config->position_tolerance_mm = 1.0F;
  config->enforce_tool_pitch = true;
  config->tool_pitch_rad = 0.0F;
  config->tool_pitch_weight_mm = 100.0F;
  config->orientation_tolerance_rad = 0.01F;
  config->max_iterations = 40U;
}

static bool config_valid(const ArmKinematicsConfig *config) {
  if (config == NULL || !finite_float(config->damping_mm) ||
      !finite_float(config->secondary_gain) ||
      !finite_float(config->max_step_rad) ||
      !finite_float(config->position_tolerance_mm) ||
      !finite_float(config->tool_pitch_rad) ||
      !finite_float(config->tool_pitch_weight_mm) ||
      !finite_float(config->orientation_tolerance_rad) ||
      config->damping_mm < 0.0F || config->secondary_gain < 0.0F ||
      config->max_step_rad <= 0.0F || config->position_tolerance_mm <= 0.0F ||
      config->tool_pitch_weight_mm < 0.0F ||
      config->orientation_tolerance_rad <= 0.0F ||
      (config->enforce_tool_pitch && config->tool_pitch_weight_mm <= 0.0F) ||
      config->max_iterations == 0U ||
      config->max_iterations > ARM_MAX_ITERATIONS) {
    return false;
  }
  for (size_t i = 0U; i < ARM_LINK_COUNT; ++i) {
    if (!finite_float(config->link_length_mm[i]) ||
        config->link_length_mm[i] <= 0.0F) {
      return false;
    }
  }
  for (size_t i = 0U; i < ARM_JOINT_COUNT; ++i) {
    if (!finite_float(config->joint_min_rad[i]) ||
        !finite_float(config->joint_max_rad[i]) ||
        !finite_float(config->preferred_rad[i]) ||
        config->joint_min_rad[i] > config->joint_max_rad[i]) {
      return false;
    }
  }
  return true;
}

static float clamp_joint(const ArmKinematicsConfig *config, size_t index,
                         float value) {
  if (value < config->joint_min_rad[index]) {
    return config->joint_min_rad[index];
  }
  if (value > config->joint_max_rad[index]) {
    return config->joint_max_rad[index];
  }
  return value;
}

static float wrap_angle(float value) {
  while (value > ARM_PI) {
    value -= ARM_TWO_PI;
  }
  while (value < -ARM_PI) {
    value += ARM_TWO_PI;
  }
  return value;
}

static float nearest_angle(float reference, float target) {
  return reference + wrap_angle(target - reference);
}

static bool within_joint_limit(const ArmKinematicsConfig *config, size_t index,
                               float value) {
  return value >= config->joint_min_rad[index] - ARM_EPSILON &&
         value <= config->joint_max_rad[index] + ARM_EPSILON;
}

static void planar_forward(const ArmKinematicsConfig *config,
                           const float q_rad[ARM_JOINT_COUNT],
                           PlanarState *state) {
  float theta[ARM_LINK_COUNT];
  float theta_sum = 0.0F;

  state->rho_mm = 0.0F;
  state->z_mm = 0.0F;
  state->tool_pitch_rad = 0.0F;
  for (size_t link = 0U; link < ARM_LINK_COUNT; ++link) {
    theta_sum += q_rad[link + 1U];
    theta[link] = theta_sum;
    state->rho_mm += config->link_length_mm[link] * cosf(theta_sum);
    state->z_mm += config->link_length_mm[link] * sinf(theta_sum);
  }
  state->tool_pitch_rad = theta_sum;

  for (size_t joint = 0U; joint < ARM_LINK_COUNT; ++joint) {
    float dr = 0.0F;
    float dz = 0.0F;
    for (size_t link = joint; link < ARM_LINK_COUNT; ++link) {
      dr -= config->link_length_mm[link] * sinf(theta[link]);
      dz += config->link_length_mm[link] * cosf(theta[link]);
    }
    state->jacobian[0][joint] = dr;
    state->jacobian[1][joint] = dz;
  }
}

/** @brief 根据五个关节角计算末端在基座坐标系中的位置。 */
bool arm_kinematics_forward(const ArmKinematicsConfig *config,
                            const float q_rad[ARM_JOINT_COUNT],
                            ArmPoint *out) {
  PlanarState state;

  if (!config_valid(config) || q_rad == NULL || out == NULL) {
    return false;
  }
  for (size_t i = 0U; i < ARM_JOINT_COUNT; ++i) {
    if (!finite_float(q_rad[i])) {
      return false;
    }
  }

  planar_forward(config, q_rad, &state);
  out->x_mm = state.rho_mm * cosf(q_rad[0]);
  out->y_mm = state.rho_mm * sinf(q_rad[0]);
  out->z_mm = state.z_mm;
  return finite_float(out->x_mm) && finite_float(out->y_mm) &&
         finite_float(out->z_mm);
}

static bool task_error(const ArmKinematicsConfig *config,
                       const float q_rad[ARM_JOINT_COUNT], float target_rho,
                       float target_z, float *task_norm,
                       float *position_error, float *orientation_error) {
  PlanarState state;
  float dr;
  float dz;
  float d_tool = 0.0F;

  planar_forward(config, q_rad, &state);
  dr = target_rho - state.rho_mm;
  dz = target_z - state.z_mm;
  if (config->enforce_tool_pitch) {
    d_tool = wrap_angle(config->tool_pitch_rad - state.tool_pitch_rad);
  }
  *position_error = hypotf(dr, dz);
  *task_norm = sqrtf(dr * dr + dz * dz +
                     (d_tool * config->tool_pitch_weight_mm) *
                         (d_tool * config->tool_pitch_weight_mm));
  if (orientation_error != NULL) {
    *orientation_error = d_tool;
  }
  return finite_float(*task_norm) && finite_float(*position_error);
}

static bool solve_3x3_inverse(float matrix[3][3], float inverse[3][3]) {
  float determinant;

  determinant = matrix[0][0] *
                    (matrix[1][1] * matrix[2][2] -
                     matrix[1][2] * matrix[2][1]) -
                matrix[0][1] *
                    (matrix[1][0] * matrix[2][2] -
                     matrix[1][2] * matrix[2][0]) +
                matrix[0][2] *
                    (matrix[1][0] * matrix[2][1] -
                     matrix[1][1] * matrix[2][0]);
  if (!finite_float(determinant) || fabsf(determinant) <= ARM_EPSILON) {
    return false;
  }

  inverse[0][0] = (matrix[1][1] * matrix[2][2] -
                   matrix[1][2] * matrix[2][1]) /
                  determinant;
  inverse[0][1] = (matrix[0][2] * matrix[2][1] -
                   matrix[0][1] * matrix[2][2]) /
                  determinant;
  inverse[0][2] = (matrix[0][1] * matrix[1][2] -
                   matrix[0][2] * matrix[1][1]) /
                  determinant;
  inverse[1][0] = (matrix[1][2] * matrix[2][0] -
                   matrix[1][0] * matrix[2][2]) /
                  determinant;
  inverse[1][1] = (matrix[0][0] * matrix[2][2] -
                   matrix[0][2] * matrix[2][0]) /
                  determinant;
  inverse[1][2] = (matrix[0][2] * matrix[1][0] -
                   matrix[0][0] * matrix[1][2]) /
                  determinant;
  inverse[2][0] = (matrix[1][0] * matrix[2][1] -
                   matrix[1][1] * matrix[2][0]) /
                  determinant;
  inverse[2][1] = (matrix[0][1] * matrix[2][0] -
                   matrix[0][0] * matrix[2][1]) /
                  determinant;
  inverse[2][2] = (matrix[0][0] * matrix[1][1] -
                   matrix[0][1] * matrix[1][0]) /
                  determinant;

  for (size_t row = 0U; row < 3U; ++row) {
    for (size_t column = 0U; column < 3U; ++column) {
      if (!finite_float(inverse[row][column])) {
        return false;
      }
    }
  }
  return true;
}

static void secondary_gradient(const ArmKinematicsConfig *config,
                               const float q_rad[ARM_JOINT_COUNT],
                               float gradient[ARM_LINK_COUNT]) {
  for (size_t joint = 0U; joint < ARM_LINK_COUNT; ++joint) {
    size_t index = joint + 1U;
    float center = 0.5F * (config->joint_min_rad[index] +
                           config->joint_max_rad[index]);
    float half_range = 0.5F * (config->joint_max_rad[index] -
                               config->joint_min_rad[index]);
    float toward_preferred = config->preferred_rad[index] - q_rad[index];
    float toward_center = 0.0F;

    if (half_range > ARM_EPSILON) {
      float normalized = (q_rad[index] - center) / half_range;
      float margin = 1.0F - normalized * normalized;
      if (margin < 0.25F) {
        float safe_margin = margin > 0.02F ? margin : 0.02F;
        toward_center = -normalized / safe_margin;
      }
    }
    gradient[joint] = toward_preferred + 0.05F * toward_center;
  }
}

static void clamp_all_joints(const ArmKinematicsConfig *config,
                             float q_rad[ARM_JOINT_COUNT]) {
  for (size_t i = 0U; i < ARM_JOINT_COUNT; ++i) {
    q_rad[i] = clamp_joint(config, i, q_rad[i]);
  }
}

static float full_position_error(const ArmKinematicsConfig *config,
                                 const float q_rad[ARM_JOINT_COUNT],
                                 const ArmPoint *target) {
  ArmPoint point;
  if (!arm_kinematics_forward(config, q_rad, &point)) {
    return INFINITY;
  }
  return sqrtf((point.x_mm - target->x_mm) * (point.x_mm - target->x_mm) +
               (point.y_mm - target->y_mm) * (point.y_mm - target->y_mm) +
               (point.z_mm - target->z_mm) * (point.z_mm - target->z_mm));
}

/** @brief 使用阻尼最小二乘迭代求解带水平末端约束的逆运动学。 */
ArmIkStatus arm_kinematics_inverse(const ArmKinematicsConfig *config,
                                   const ArmPoint *target,
                                   const float seed_rad[ARM_JOINT_COUNT],
                                   float q_rad[ARM_JOINT_COUNT],
                                   ArmIkResult *result) {
  float target_rho;
  float target_yaw;
  float planar_target_z;
  float current_error;
  float current_position_error;
  float current_orientation_error;
  uint8_t iterations = 0U;

  if (result != NULL) {
    result->status = ARM_IK_BAD_ARGUMENT;
    result->position_error_mm = INFINITY;
    result->orientation_error_rad = INFINITY;
    result->iterations = 0U;
  }
  if (!config_valid(config) || target == NULL || q_rad == NULL ||
      !finite_float(target->x_mm) || !finite_float(target->y_mm) ||
      !finite_float(target->z_mm)) {
    return ARM_IK_BAD_ARGUMENT;
  }

  for (size_t i = 0U; i < ARM_JOINT_COUNT; ++i) {
    q_rad[i] = seed_rad != NULL ? seed_rad[i] : config->preferred_rad[i];
    if (!finite_float(q_rad[i])) {
      return ARM_IK_BAD_ARGUMENT;
    }
  }
  clamp_all_joints(config, q_rad);

  target_rho = hypotf(target->x_mm, target->y_mm);
  planar_target_z = target->z_mm;
  if (!finite_float(target_rho)) {
    return ARM_IK_BAD_ARGUMENT;
  }

  if (target_rho > ARM_EPSILON) {
    target_yaw = nearest_angle(q_rad[0], atan2f(target->y_mm, target->x_mm));
    if (!within_joint_limit(config, 0U, target_yaw)) {
      return ARM_IK_UNREACHABLE;
    }
    q_rad[0] = target_yaw;
  }

  if (!task_error(config, q_rad, target_rho, planar_target_z, &current_error,
                  &current_position_error, &current_orientation_error)) {
    return ARM_IK_NUMERIC_ERROR;
  }

  for (iterations = 0U; iterations < config->max_iterations; ++iterations) {
    PlanarState state;
    float error[3] = {0.0F, 0.0F, 0.0F};
    float task_jacobian[3][ARM_LINK_COUNT] = {{0.0F}};
    float gram[3][3] = {{0.0F}};
    float inverse[3][3] = {{0.0F}};
    float pseudoinverse[ARM_LINK_COUNT][3] = {{0.0F}};
    float gradient[ARM_LINK_COUNT];
    float delta[ARM_LINK_COUNT];
    float candidate[ARM_JOINT_COUNT];
    float candidate_error;
    float candidate_position_error;
    float candidate_orientation_error;
    float damping = config->damping_mm > ARM_MIN_DAMPING_MM
                        ? config->damping_mm
                        : ARM_MIN_DAMPING_MM;
    bool accepted = false;

    if (current_position_error <= config->position_tolerance_mm &&
        (!config->enforce_tool_pitch ||
         fabsf(current_orientation_error) <=
             config->orientation_tolerance_rad)) {
      break;
    }

    planar_forward(config, q_rad, &state);
    error[0] = target_rho - state.rho_mm;
    error[1] = planar_target_z - state.z_mm;
    error[2] = config->enforce_tool_pitch
                   ? wrap_angle(config->tool_pitch_rad -
                                state.tool_pitch_rad) *
                         config->tool_pitch_weight_mm
                   : 0.0F;
    for (size_t joint = 0U; joint < ARM_LINK_COUNT; ++joint) {
      task_jacobian[0][joint] = state.jacobian[0][joint];
      task_jacobian[1][joint] = state.jacobian[1][joint];
      task_jacobian[2][joint] =
          config->enforce_tool_pitch ? config->tool_pitch_weight_mm : 0.0F;
    }

    for (size_t row = 0U; row < 3U; ++row) {
      for (size_t column = 0U; column < 3U; ++column) {
        for (size_t joint = 0U; joint < ARM_LINK_COUNT; ++joint) {
          gram[row][column] += task_jacobian[row][joint] *
                               task_jacobian[column][joint];
        }
        if (row == column) {
          gram[row][column] += damping * damping;
        }
      }
    }
    if (!solve_3x3_inverse(gram, inverse)) {
      return ARM_IK_NUMERIC_ERROR;
    }

    for (size_t joint = 0U; joint < ARM_LINK_COUNT; ++joint) {
      for (size_t row = 0U; row < 3U; ++row) {
        for (size_t axis = 0U; axis < 3U; ++axis) {
          pseudoinverse[joint][row] +=
              task_jacobian[axis][joint] * inverse[axis][row];
        }
      }
      delta[joint] = 0.0F;
      for (size_t row = 0U; row < 3U; ++row) {
        delta[joint] += pseudoinverse[joint][row] * error[row];
      }
    }

    if (config->secondary_gain > 0.0F) {
      secondary_gradient(config, q_rad, gradient);
      for (size_t row = 0U; row < ARM_LINK_COUNT; ++row) {
        float projected_gradient = 0.0F;
        for (size_t column = 0U; column < ARM_LINK_COUNT; ++column) {
          float projection = 0.0F;
          for (size_t axis = 0U; axis < 3U; ++axis) {
            projection += pseudoinverse[row][axis] *
                          task_jacobian[axis][column];
          }
          projected_gradient += projection * gradient[column];
        }
        delta[row] +=
            config->secondary_gain * (gradient[row] - projected_gradient);
      }
    }

    {
      float largest_step = 0.0F;
      for (size_t joint = 0U; joint < ARM_LINK_COUNT; ++joint) {
        float magnitude = fabsf(delta[joint]);
        if (magnitude > largest_step) {
          largest_step = magnitude;
        }
      }
      if (largest_step > config->max_step_rad) {
        float scale = config->max_step_rad / largest_step;
        for (size_t joint = 0U; joint < ARM_LINK_COUNT; ++joint) {
          delta[joint] *= scale;
        }
      }
    }

    for (size_t line = 0U; line < ARM_MAX_LINE_SEARCH; ++line) {
      float scale = 1.0F / (float)(1U << line);
      for (size_t i = 0U; i < ARM_JOINT_COUNT; ++i) {
        candidate[i] = q_rad[i];
      }
      for (size_t joint = 0U; joint < ARM_LINK_COUNT; ++joint) {
        candidate[joint + 1U] = clamp_joint(
            config, joint + 1U, q_rad[joint + 1U] + delta[joint] * scale);
      }
      if (!task_error(config, candidate, target_rho, planar_target_z,
                      &candidate_error, &candidate_position_error,
                      &candidate_orientation_error)) {
        return ARM_IK_NUMERIC_ERROR;
      }
      if (candidate_error < current_error) {
        for (size_t i = 0U; i < ARM_JOINT_COUNT; ++i) {
          q_rad[i] = candidate[i];
        }
        current_error = candidate_error;
        current_position_error = candidate_position_error;
        current_orientation_error = candidate_orientation_error;
        accepted = true;
        break;
      }
    }
    if (!accepted) {
      break;
    }
  }

  if (result != NULL) {
    result->position_error_mm = full_position_error(config, q_rad, target);
    result->orientation_error_rad = config->enforce_tool_pitch
                                        ? fabsf(current_orientation_error)
                                        : 0.0F;
    result->iterations = iterations;
  }
  if (current_position_error <= config->position_tolerance_mm &&
      (!config->enforce_tool_pitch ||
       fabsf(current_orientation_error) <= config->orientation_tolerance_rad)) {
    if (result != NULL) {
      result->status = ARM_IK_OK;
    }
    return ARM_IK_OK;
  }
  if (result != NULL) {
    result->status = ARM_IK_UNREACHABLE;
  }
  return ARM_IK_UNREACHABLE;
}
