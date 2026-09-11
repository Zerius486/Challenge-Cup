#ifndef LEGACY_CAN_BRIDGE_H
#define LEGACY_CAN_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_frame.h"

#define LEGACY_CAN_BRIDGE_FRAME_SIZE 15U

bool legacy_can_bridge_decode(const uint8_t *data, size_t length,
                              CanFrame *out);
size_t legacy_can_bridge_encode(const CanFrame *frame,
                                uint8_t out[LEGACY_CAN_BRIDGE_FRAME_SIZE]);
bool legacy_can_bridge_is_safe_stop(const CanFrame *frame);

#endif
