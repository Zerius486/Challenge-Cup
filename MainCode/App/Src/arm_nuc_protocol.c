#include "arm_nuc_protocol.h"

#include <limits.h>
#include <string.h>

static bool delimiter(char c) {
  return c == '\0' || c == ',' || c == '/' || c == '_' || c == ' ' ||
         c == '\t' || c == '\r' || c == '\n';
}

static bool parse_i32(const char **cursor, int32_t *value) {
  const char *p = *cursor;
  int sign = 1;
  int64_t limit = INT32_MAX;
  int64_t result = 0;
  bool any = false;

  if (*p == '+' || *p == '-') {
    if (*p++ == '-') {
      sign = -1;
      limit = -(int64_t)INT32_MIN;
    }
  }
  while (*p >= '0' && *p <= '9') {
    int64_t digit = (int64_t)(*p - '0');
    any = true;
    if (result > (limit - digit) / 10) {
      return false;
    }
    result = result * 10 + digit;
    ++p;
  }
  if (!any) {
    return false;
  }
  *cursor = p;
  *value = (int32_t)(sign < 0 ? -result : result);
  return true;
}

static bool parse_u16(const char **cursor, uint16_t *value) {
  int32_t signed_value;
  if (!parse_i32(cursor, &signed_value) || signed_value < 0 ||
      signed_value > UINT16_MAX) {
    return false;
  }
  *value = (uint16_t)signed_value;
  return true;
}

static const char *find_token(const char *text, const char *token) {
  const char *p = text;
  while ((p = strstr(p, token)) != NULL) {
    if (p == text || delimiter(p[-1])) {
      return p;
    }
    ++p;
  }
  return NULL;
}

/** @brief 解析 NUC 的 PXYZ/JZH/ST:P ASCII 指令。 */
bool arm_nuc_parse(const uint8_t *data, size_t length, ArmNucCommand *out) {
  char text[96];
  const char *position;
  const char *gripper;
  const char *p;
  bool has_position = false;
  bool has_gripper = false;

  if (data == NULL || out == NULL || length == 0U || length >= sizeof(text)) {
    return false;
  }
  memcpy(text, data, length);
  text[length] = '\0';
  memset(out, 0, sizeof(*out));

  position = find_token(text, "PXYZ:");
  if (position != NULL) {
    p = position + 5U;
    if (!parse_i32(&p, &out->x_centi_mm) || *p++ != ',' ||
        !parse_i32(&p, &out->y_centi_mm) || *p++ != ',' ||
        !parse_i32(&p, &out->z_centi_mm) || *p++ != ',') {
      return false;
    }
    has_position = true;
  }

  gripper = find_token(text, "JZH_");
  if (gripper != NULL) {
    p = gripper + 4U;
    if (!parse_i32(&p, &out->gripper_millirad) || *p++ != '_' ||
        !parse_u16(&p, &out->gripper_current_ma) || !delimiter(*p) ||
        out->gripper_millirad < -2000 || out->gripper_millirad > 2000) {
      return false;
    }
    has_gripper = true;
  }

  if (!has_position && !has_gripper &&
      (strcmp(text, "ST:P/") == 0 || strcmp(text, "ST:P") == 0 ||
       strcmp(text, "ZT/") == 0 || strcmp(text, "ZT") == 0)) {
    out->type = ARM_NUC_STOP;
    return true;
  }
  if (!has_position && !has_gripper) {
    return false;
  }
  out->has_position = has_position;
  out->has_gripper = has_gripper;
  out->type = has_position && has_gripper
                  ? ARM_NUC_BUNDLE
                  : (has_position ? ARM_NUC_POSITION : ARM_NUC_GRIPPER);
  return true;
}
