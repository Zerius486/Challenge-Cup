#include "app_bridge.h"
#include "udp_transport.h"

__attribute__((weak)) void app_bridge_on_can_frame(uint8_t bus,
                                                    const CanFrame *frame,
                                                    uint32_t now_ms) {
  (void)bus;
  (void)frame;
  (void)now_ms;
}

__attribute__((weak)) void app_bridge_on_can_error(uint8_t bus,
                                                   uint32_t error) {
  (void)bus;
  (void)error;
}

__attribute__((weak)) void udp_transport_queue_can_from_isr(
    uint8_t bus, const CanFrame *frame) {
  (void)bus;
  (void)frame;
}
