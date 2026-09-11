#ifndef STANDALONE_RUNTIME_H
#define STANDALONE_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

void standalone_runtime_init(uint32_t peripherals);
void standalone_heartbeat_poll(uint32_t now_ms);
void standalone_set_fault(bool active);

#endif
