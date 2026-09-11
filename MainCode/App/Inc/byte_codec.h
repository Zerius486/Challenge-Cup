#ifndef BYTE_CODEC_H
#define BYTE_CODEC_H

#include <stdint.h>
#include <string.h>

static inline uint16_t codec_read_u16_le(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint16_t codec_read_u16_be(const uint8_t *p) {
  return ((uint16_t)p[0] << 8) | (uint16_t)p[1];
}

static inline uint32_t codec_read_u32_le(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline uint32_t codec_read_u32_be(const uint8_t *p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
         ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static inline void codec_write_u16_le(uint8_t *p, uint16_t value) {
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8);
}

static inline void codec_write_u16_be(uint8_t *p, uint16_t value) {
  p[0] = (uint8_t)(value >> 8);
  p[1] = (uint8_t)value;
}

static inline void codec_write_u32_le(uint8_t *p, uint32_t value) {
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8);
  p[2] = (uint8_t)(value >> 16);
  p[3] = (uint8_t)(value >> 24);
}

static inline void codec_write_u32_be(uint8_t *p, uint32_t value) {
  p[0] = (uint8_t)(value >> 24);
  p[1] = (uint8_t)(value >> 16);
  p[2] = (uint8_t)(value >> 8);
  p[3] = (uint8_t)value;
}

static inline float codec_read_f32_le(const uint8_t *p) {
  uint32_t bits = codec_read_u32_le(p);
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

static inline float codec_read_f32_be(const uint8_t *p) {
  uint32_t bits = codec_read_u32_be(p);
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

static inline void codec_write_f32_le(uint8_t *p, float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  codec_write_u32_le(p, bits);
}

#endif
