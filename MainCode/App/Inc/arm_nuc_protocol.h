#ifndef ARM_NUC_PROTOCOL_H
#define ARM_NUC_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Small ASCII command set used by the NUC arm task.  Coordinates are sent in
 * 0.01 mm, while the gripper position is sent in 0.001 rad. */
typedef enum {
  ARM_NUC_NONE = 0,
  ARM_NUC_POSITION,
  ARM_NUC_GRIPPER,
  ARM_NUC_BUNDLE,
  ARM_NUC_STOP
} ArmNucCommandType;

typedef struct {
  ArmNucCommandType type;
  bool has_position;
  bool has_gripper;
  int32_t x_centi_mm;
  int32_t y_centi_mm;
  int32_t z_centi_mm;
  int32_t gripper_millirad;
  uint16_t gripper_current_ma;
} ArmNucCommand;

bool arm_nuc_parse(const uint8_t *data, size_t length, ArmNucCommand *out);

#endif
