#include "force_sensor.h"

#include <string.h>

#include "byte_codec.h"

enum {
  WAIT_START_AA = 0,
  WAIT_START_55,
  WAIT_COMMAND,
  READ_PAYLOAD,
  WAIT_CR,
  WAIT_LF,
  WAIT_VARIABLE_LF
};

#define VARIABLE_LENGTH 0xFFU

static uint8_t expected_payload(uint8_t command) {
  switch (command) {
    case FORCE_CMD_START_STREAM_1KHZ:
    case FORCE_CMD_READ_ONCE:
    case FORCE_CMD_INITIALIZE:
    case FORCE_CMD_READ_KG:
    case FORCE_CMD_READ_RAW_VOLTAGE:
    case FORCE_CMD_READ_NEWTON:
      return 24U;
    case FORCE_CMD_READ_SERIAL:
      return 16U;
    case FORCE_CMD_SET_BAUD:
    case FORCE_CMD_READ_FIRMWARE:
    case FORCE_CMD_ENTER_DEBUG:
      return VARIABLE_LENGTH;
    default:
      return 0U;
  }
}

static void restart_from_byte(ForceSensorParser *parser, uint8_t byte) {
  parser->state = (byte == 0xAAU) ? WAIT_START_55 : WAIT_START_AA;
  parser->payload_length = 0U;
}

/** @brief 清空力传感器帧解析器，准备接收新字节流。 */
void force_sensor_parser_init(ForceSensorParser *parser) {
  if (parser != NULL) {
    memset(parser, 0, sizeof(*parser));
  }
}

/** @brief 输入一个字节并在帧完整时返回解析结果。 */
ForceParseResult force_sensor_parser_feed(ForceSensorParser *parser,
                                          uint8_t byte,
                                          ForceSensorFrame *out) {
  if (parser == NULL || out == NULL) {
    return FORCE_PARSE_ERROR;
  }

  switch (parser->state) {
    case WAIT_START_AA:
      if (byte == 0xAAU) {
        parser->state = WAIT_START_55;
      }
      return FORCE_PARSE_NONE;

    case WAIT_START_55:
      if (byte == 0x55U) {
        parser->state = WAIT_COMMAND;
      } else if (byte != 0xAAU) {
        parser->state = WAIT_START_AA;
      }
      return FORCE_PARSE_NONE;

    case WAIT_COMMAND:
      parser->command = byte;
      parser->payload_length = 0U;
      parser->expected_payload_length = expected_payload(byte);
      parser->state = (parser->expected_payload_length == 0U)
                          ? WAIT_CR
                          : READ_PAYLOAD;
      return FORCE_PARSE_NONE;

    case READ_PAYLOAD:
      if (parser->expected_payload_length == VARIABLE_LENGTH && byte == 0x0DU) {
        parser->state = WAIT_VARIABLE_LF;
        return FORCE_PARSE_NONE;
      }
      if (parser->payload_length >= FORCE_SENSOR_MAX_PAYLOAD) {
        restart_from_byte(parser, byte);
        return FORCE_PARSE_ERROR;
      }
      parser->payload[parser->payload_length++] = byte;
      if (parser->expected_payload_length != VARIABLE_LENGTH &&
          parser->payload_length == parser->expected_payload_length) {
        parser->state = WAIT_CR;
      }
      return FORCE_PARSE_NONE;

    case WAIT_CR:
      if (byte == 0x0DU) {
        parser->state = WAIT_LF;
        return FORCE_PARSE_NONE;
      }
      restart_from_byte(parser, byte);
      return FORCE_PARSE_ERROR;

    case WAIT_VARIABLE_LF:
      if (byte == 0x0AU) {
        parser->state = WAIT_START_AA;
        out->command = parser->command;
        out->payload_length = parser->payload_length;
        memcpy(out->payload, parser->payload, parser->payload_length);
        return FORCE_PARSE_FRAME;
      }
      if (parser->payload_length + 2U > FORCE_SENSOR_MAX_PAYLOAD) {
        restart_from_byte(parser, byte);
        return FORCE_PARSE_ERROR;
      }
      parser->payload[parser->payload_length++] = 0x0DU;
      parser->payload[parser->payload_length++] = byte;
      parser->state = READ_PAYLOAD;
      return FORCE_PARSE_NONE;

    case WAIT_LF:
      if (byte == 0x0AU) {
        parser->state = WAIT_START_AA;
        out->command = parser->command;
        out->payload_length = parser->payload_length;
        memcpy(out->payload, parser->payload, parser->payload_length);
        return FORCE_PARSE_FRAME;
      }
      restart_from_byte(parser, byte);
      return FORCE_PARSE_ERROR;

    default:
      force_sensor_parser_init(parser);
      return FORCE_PARSE_ERROR;
  }
}

/** @brief 构造带长度、命令字和校验的力传感器命令帧。 */
size_t force_sensor_build_command(ForceSensorCommand command,
                                  const uint8_t *payload,
                                  uint8_t payload_length,
                                  uint8_t *out, size_t capacity) {
  size_t frame_length = (size_t)payload_length + 5U;
  if (out == NULL || payload_length > FORCE_SENSOR_MAX_PAYLOAD ||
      capacity < frame_length ||
      (payload_length > 0U && payload == NULL)) {
    return 0U;
  }
  out[0] = 0xAAU;
  out[1] = 0x55U;
  out[2] = (uint8_t)command;
  if (payload_length > 0U) {
    memcpy(&out[3], payload, payload_length);
  }
  out[3U + payload_length] = 0x0DU;
  out[4U + payload_length] = 0x0AU;
  return frame_length;
}

/** @brief 构造无参数的力传感器命令。 */
size_t force_sensor_build_simple_command(ForceSensorCommand command,
                                         uint8_t out[5]) {
  return force_sensor_build_command(command, NULL, 0U, out, 5U);
}

/** @brief 构造切换力传感器波特率的命令。 */
size_t force_sensor_build_baud_command(ForceSensorBaudCode baud,
                                       uint8_t out[6]) {
  uint8_t value = (uint8_t)baud;
  if (baud < FORCE_BAUD_460800 || baud > FORCE_BAUD_921600) {
    return 0U;
  }
  return force_sensor_build_command(FORCE_CMD_SET_BAUD, &value, 1U, out, 6U);
}

/** @brief 按默认字节序解析六维力数据。 */
bool force_sensor_decode_sample(const ForceSensorFrame *frame,
                                ForceSensorSample *out) {
  return force_sensor_decode_sample_order(frame,
                                          FORCE_FLOAT_ORDER_MANUAL_EXAMPLE,
                                          out);
}

/** @brief 按指定浮点字节序解析六维力数据。 */
bool force_sensor_decode_sample_order(const ForceSensorFrame *frame,
                                      ForceSensorFloatOrder order,
                                      ForceSensorSample *out) {
  float (*read_float)(const uint8_t *);

  if (frame == NULL || out == NULL || frame->payload_length != 24U ||
      (order != FORCE_FLOAT_ORDER_MANUAL_EXAMPLE &&
       order != FORCE_FLOAT_ORDER_BIG_ENDIAN)) {
    return false;
  }
  switch (frame->command) {
    case FORCE_CMD_START_STREAM_1KHZ:
    case FORCE_CMD_READ_ONCE:
    case FORCE_CMD_INITIALIZE:
    case FORCE_CMD_READ_KG:
    case FORCE_CMD_READ_RAW_VOLTAGE:
    case FORCE_CMD_READ_NEWTON:
      break;
    default:
      return false;
  }

  read_float = (order == FORCE_FLOAT_ORDER_BIG_ENDIAN)
                   ? codec_read_f32_be
                   : codec_read_f32_le;
  out->fx = read_float(&frame->payload[0]);
  out->fy = read_float(&frame->payload[4]);
  out->fz = read_float(&frame->payload[8]);
  out->mx = read_float(&frame->payload[12]);
  out->my = read_float(&frame->payload[16]);
  out->mz = read_float(&frame->payload[20]);
  return true;
}
