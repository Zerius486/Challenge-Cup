#ifndef UDP_TRANSPORT_H
#define UDP_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

bool udp_transport_init(void);
void udp_transport_poll(uint32_t now_ms);
void udp_transport_queue_can_from_isr(uint8_t bus, const CanFrame *frame);

#endif
