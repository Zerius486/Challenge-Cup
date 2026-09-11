#ifndef CAN_FRAME_H
#define CAN_FRAME_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t id;
  uint8_t dlc;
  bool is_extended;
  bool is_remote;
  uint8_t data[8];
} CanFrame;

#endif
