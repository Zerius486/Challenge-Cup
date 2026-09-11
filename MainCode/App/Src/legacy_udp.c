#include "legacy_udp.h"

#include <limits.h>
#include <string.h>

static bool is_boundary(char c) {
  return c == '\0' || c == ',' || c == '/' || c == ':' || c == '_' ||
         c == ' ' || c == '\t';
}

static const char *find_prefixed(const char *text, const char *prefix) {
  const char *p = text;
  while ((p = strstr(p, prefix)) != NULL) {
    if (p == text || is_boundary(p[-1])) {
      return p;
    }
    ++p;
  }
  return NULL;
}

static bool has_terminated_token(const char *text, const char *token,
                                 char terminator) {
  size_t token_length = strlen(token);
  const char *p = text;

  while ((p = strstr(p, token)) != NULL) {
    if ((p == text || is_boundary(p[-1])) && p[token_length] == terminator) {
      return true;
    }
    ++p;
  }
  return false;
}

static bool parse_i16(const char **cursor, int16_t *out) {
  const char *p = *cursor;
  int sign = 1;
  int32_t limit = INT16_MAX;
  int32_t value = 0;
  bool any = false;

  if (*p == '+' || *p == '-') {
    if (*p++ == '-') {
      sign = -1;
      limit = -(int32_t)INT16_MIN;
    }
  }
  while (*p >= '0' && *p <= '9') {
    int32_t digit = (int32_t)(*p - '0');
    any = true;
    if (value > (limit - digit) / 10) {
      return false;
    }
    value = value * 10 + digit;
    ++p;
  }
  if (!any) {
    return false;
  }
  value *= sign;
  *cursor = p;
  *out = (int16_t)value;
  return true;
}

/** @brief 解析旧版 ASCII UDP 中的急停、履带和灯光字段。 */
bool legacy_udp_parse(const uint8_t *datagram, size_t length,
                      LegacyUdpCommand *out) {
  char text[192];
  const char *track;
  const char *light;
  const char *p;

  if (datagram == NULL || out == NULL || length == 0U ||
      length >= sizeof(text) || datagram[length - 1U] != '/') {
    return false;
  }
  memcpy(text, datagram, length);
  text[length] = '\0';
  memset(out, 0, sizeof(*out));

  /* ZT_<pwm>/ is a drill command in the NUC protocol, not a stop command. */
  out->stop_requested = has_terminated_token(text, "ZT", '/') ||
                        has_terminated_token(text, "ST:P", '/');

  track = find_prefixed(text, "L_");
  if (track != NULL) {
    p = track + 2;
    if (!parse_i16(&p, &out->left_track) || (*p != '_' && *p != ',')) {
      return false;
    }
    ++p;
    if (!parse_i16(&p, &out->right_track) || !is_boundary(*p)) {
      return false;
    }
    out->has_track = true;
  }

  light = find_prefixed(text, "D:");
  {
    const char *legacy_light = find_prefixed(text, "D_");
    if (light == NULL ||
        (legacy_light != NULL && legacy_light < light)) {
      light = legacy_light;
    }
  }
  if (light != NULL) {
    if ((light[2] != '0' && light[2] != '1') || !is_boundary(light[3])) {
      return false;
    }
    out->has_light = true;
    out->light_on = (uint8_t)(light[2] - '0');
  }

  return out->stop_requested || out->has_track || out->has_light;
}
