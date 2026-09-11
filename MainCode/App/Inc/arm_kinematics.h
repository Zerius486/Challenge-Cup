#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include <stdbool.h>
#include <stdint.h>

#define ARM_JOINT_COUNT 5U
#define ARM_LINK_COUNT 4U

typedef struct {
  float x_mm;
  float y_mm;
  float z_mm;
} ArmPoint;

/*
 * q[0] is the base yaw. q[1]..q[4] are the four coplanar pitch joints.
 * All angles are radians and all lengths/positions are millimetres.
 */
typedef struct {
  float link_length_mm[ARM_LINK_COUNT];
  float joint_min_rad[ARM_JOINT_COUNT];
  float joint_max_rad[ARM_JOINT_COUNT];
  float preferred_rad[ARM_JOINT_COUNT];
  float damping_mm;
  float secondary_gain;
  float max_step_rad;
  float position_tolerance_mm;
  bool enforce_tool_pitch;
  float tool_pitch_rad;
  float tool_pitch_weight_mm;
  float orientation_tolerance_rad;
  uint8_t max_iterations;
} ArmKinematicsConfig;

typedef enum {
  ARM_IK_OK = 0,
  ARM_IK_BAD_ARGUMENT,
  ARM_IK_UNREACHABLE,
  ARM_IK_NUMERIC_ERROR
} ArmIkStatus;

typedef struct {
  ArmIkStatus status;
  float position_error_mm;
  float orientation_error_rad;
  uint8_t iterations;
} ArmIkResult;

/* Conservative starting limits, centred at the encoder-midpoint software
 * zero. They are for initial commissioning only and must be checked against
 * the actual arm before powered motion. */
void arm_kinematics_config_default(ArmKinematicsConfig *config);

bool arm_kinematics_forward(const ArmKinematicsConfig *config,
                            const float q_rad[ARM_JOINT_COUNT],
                            ArmPoint *out);

/*
 * Solve a target in the arm-base frame (z=0 at the yaw/pitch base).
 * When enforce_tool_pitch is true, q2+q3+q4+q5 is constrained to
 * tool_pitch_rad; the current arm uses tool_pitch_rad=0 for a horizontal l4.
 * seed_rad is normally the last commanded or measured joint pose.
 * preferred_rad and secondary_gain provide a small null-space posture bias.
 */
ArmIkStatus arm_kinematics_inverse(const ArmKinematicsConfig *config,
                                   const ArmPoint *target,
                                   const float seed_rad[ARM_JOINT_COUNT],
                                   float q_rad[ARM_JOINT_COUNT],
                                   ArmIkResult *result);

#endif
