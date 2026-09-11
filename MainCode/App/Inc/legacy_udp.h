#ifndef LEGACY_UDP_H
#define LEGACY_UDP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  bool stop_requested;
  bool has_track;
  int16_t left_track;
  int16_t right_track;
  bool has_light;
  uint8_t light_on;
} LegacyUdpCommand;

/* Parses legacy stop strings and documented L_/D_ (or D:) fields. */
bool legacy_udp_parse(const uint8_t *datagram, size_t length,
                      LegacyUdpCommand *out);

#endif
