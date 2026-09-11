#ifndef APP_BRIDGE_H
#define APP_BRIDGE_H

#include <stddef.h>
#include <stdint.h>

#include "can_frame.h"

void app_bridge_init(void);
void app_bridge_poll(uint32_t now_ms);
void app_bridge_on_can_frame(uint8_t bus, const CanFrame *frame,
                             uint32_t now_ms);
void app_bridge_on_can_error(uint8_t bus, uint32_t error);
size_t app_bridge_handle_datagram(const uint8_t *datagram, size_t length,
                                  uint32_t now_ms, uint8_t *response,
                                  size_t response_capacity);
size_t app_bridge_build_telemetry(uint32_t now_ms, uint8_t *payload,
                                  size_t capacity);
size_t app_bridge_build_arm_status(uint32_t now_ms, uint8_t *payload,
                                   size_t capacity);

#endif
