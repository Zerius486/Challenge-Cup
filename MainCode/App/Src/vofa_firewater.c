#include "vofa_firewater.h"

#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "app_bridge.h"
#include "app_config.h"
#include "board.h"
#include "byte_codec.h"

/*
 * FireWater 是最简单的 CSV 流格式：一行数据以换行结束，VOFA+ 按列显示。
 * 这里固定列数，便于调试时长期保持曲线颜色和名称不变。
 */
static uint32_t last_send_ms;
static uint32_t send_count;
/* 仅由主循环调用，放在静态区避免占用 1 KB 主栈。 */
static char tx_line[APP_VOFA_TX_BUFFER_SIZE];

static bool append_text(char *buffer, size_t capacity, size_t *used,
                        const char *text) {
  int written;
  if (buffer == NULL || used == NULL || text == NULL || *used >= capacity) {
    return false;
  }
  written = snprintf(&buffer[*used], capacity - *used, "%s", text);
  if (written < 0 || (size_t)written >= capacity - *used) {
    return false;
  }
  *used += (size_t)written;
  return true;
}

static bool append_separator(char *buffer, size_t capacity, size_t *used) {
  return append_text(buffer, capacity, used, ",");
}

static bool append_u32(char *buffer, size_t capacity, size_t *used,
                       uint32_t value) {
  int written;
  if (buffer == NULL || used == NULL || *used >= capacity) {
    return false;
  }
  written = snprintf(&buffer[*used], capacity - *used, "%lu",
                     (unsigned long)value);
  if (written < 0 || (size_t)written >= capacity - *used) {
    return false;
  }
  *used += (size_t)written;
  return true;
}

static bool append_float3(char *buffer, size_t capacity, size_t *used,
                          float value) {
  int written;
  long scaled;
  long magnitude;
  long whole;
  long fraction;
  if (buffer == NULL || used == NULL || *used >= capacity) {
    return false;
  }
  /* Avoid floating-point printf: fixed-point conversion is smaller on F407. */
  if (!isfinite(value)) {
    return append_text(buffer, capacity, used, "nan");
  }
  scaled = lroundf(value * 1000.0F);
  magnitude = labs(scaled);
  whole = magnitude / 1000L;
  fraction = magnitude % 1000L;
  written = snprintf(&buffer[*used], capacity - *used,
                     scaled < 0L ? "-%ld.%03ld" : "%ld.%03ld", whole,
                     fraction);
  if (written < 0 || (size_t)written >= capacity - *used) {
    return false;
  }
  *used += (size_t)written;
  return true;
}

static bool append_float_field(char *buffer, size_t capacity, size_t *used,
                               float value, bool *first) {
  if (!*first && !append_separator(buffer, capacity, used)) {
    return false;
  }
  *first = false;
  return append_float3(buffer, capacity, used, value);
}

static bool append_u32_field(char *buffer, size_t capacity, size_t *used,
                             uint32_t value, bool *first) {
  if (!*first && !append_separator(buffer, capacity, used)) {
    return false;
  }
  *first = false;
  return append_u32(buffer, capacity, used, value);
}

/**
 * @brief 将主控状态快照转换为 VOFA+ FireWater CSV 并发送。
 * @param now_ms 当前系统毫秒计数。
 */
void vofa_firewater_poll(uint32_t now_ms) {
  uint8_t telemetry[68];
  uint8_t arm_status[64];
  char *line = tx_line;
  size_t used = 0U;
  bool first = true;
  uint32_t flags;
  uint32_t can_error_flags;

  if ((uint32_t)(now_ms - last_send_ms) < APP_VOFA_PERIOD_MS) {
    return;
  }
  if (app_bridge_build_telemetry(now_ms, telemetry, sizeof(telemetry)) == 0U ||
      app_bridge_build_arm_status(now_ms, arm_status, sizeof(arm_status)) ==
          0U) {
    return;
  }

  flags = codec_read_u32_le(&telemetry[4]);
  can_error_flags = (flags >> 16) & 0x03U;

  /* 39 列：时间、链路状态、力、两类电机反馈、机械臂目标和误差。 */
  if (!append_u32_field(line, sizeof(tx_line), &used, now_ms, &first) ||
      !append_u32_field(line, sizeof(tx_line), &used, ++send_count, &first) ||
      !append_u32_field(line, sizeof(tx_line), &used, flags, &first) ||
      !append_u32_field(line, sizeof(tx_line), &used, codec_read_u32_le(&telemetry[8]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&telemetry[12]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&telemetry[16]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&telemetry[20]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&telemetry[24]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&telemetry[28]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&telemetry[32]), &first) ||
      !append_u32_field(line, sizeof(tx_line), &used, codec_read_u32_le(&telemetry[36]), &first) ||
      !append_u32_field(line, sizeof(tx_line), &used, telemetry[40], &first) ||
      !append_u32_field(line, sizeof(tx_line), &used, telemetry[41], &first) ||
      !append_u32_field(line, sizeof(tx_line), &used, codec_read_u16_le(&telemetry[42]), &first) ||
      !append_u32_field(line, sizeof(tx_line), &used, codec_read_u32_le(&telemetry[44]), &first) ||
      !append_u32_field(line, sizeof(tx_line), &used, telemetry[48], &first) ||
      !append_u32_field(line, sizeof(tx_line), &used, telemetry[49], &first) ||
      !append_u32_field(line, sizeof(tx_line), &used, telemetry[50], &first) ||
      !append_u32_field(line, sizeof(tx_line), &used, telemetry[51], &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&telemetry[52]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&telemetry[56]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&telemetry[60]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&telemetry[64]), &first) ||
      !append_u32_field(line, sizeof(tx_line), &used, arm_status[4], &first) ||
      !append_u32_field(line, sizeof(tx_line), &used, arm_status[5], &first) ||
      !append_u32_field(line, sizeof(tx_line), &used, codec_read_u16_le(&arm_status[6]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&arm_status[8]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&arm_status[12]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&arm_status[16]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&arm_status[20]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&arm_status[24]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&arm_status[28]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&arm_status[32]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&arm_status[36]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&arm_status[40]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&arm_status[44]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&arm_status[52]), &first) ||
      !append_float_field(line, sizeof(tx_line), &used, codec_read_f32_le(&arm_status[56]), &first) ||
      !append_u32_field(line, sizeof(tx_line), &used, can_error_flags, &first)) {
    return;
  }
  if (used + 2U >= sizeof(tx_line)) {
    return;
  }
  line[used++] = '\r';
  line[used++] = '\n';
  if (board_vofa_uart_send((const uint8_t *)line, (uint16_t)used, 10U)) {
    last_send_ms = now_ms;
  }
}
