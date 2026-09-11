#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FORCE_SENSOR_MAX_PAYLOAD 32U

typedef enum {
  FORCE_CMD_STOP_STREAM = 0x01,
  FORCE_CMD_START_STREAM_1KHZ = 0x02,
  FORCE_CMD_READ_ONCE = 0x03,
  FORCE_CMD_SET_BAUD = 0x04,
  FORCE_CMD_READ_SERIAL = 0x05,
  FORCE_CMD_INITIALIZE = 0x06,
  FORCE_CMD_READ_FIRMWARE = 0x07,
  FORCE_CMD_TARE = 0x30,
  FORCE_CMD_EXIT_DEBUG = 0x31,
  FORCE_CMD_ENTER_DEBUG = 0x32,
  FORCE_CMD_READ_KG = 0x33,
  FORCE_CMD_READ_RAW_VOLTAGE = 0x34,
  FORCE_CMD_READ_NEWTON = 0x35
} ForceSensorCommand;

typedef enum {
  FORCE_BAUD_460800 = 0x01,
  FORCE_BAUD_691200 = 0x02,
  FORCE_BAUD_921600 = 0x03
} ForceSensorBaudCode;

typedef struct {
  uint8_t command;
  uint8_t payload[FORCE_SENSOR_MAX_PAYLOAD];
  uint8_t payload_length;
} ForceSensorFrame;

typedef struct {
  float fx;
  float fy;
  float fz;
  float mx;
  float my;
  float mz;
} ForceSensorSample;

typedef enum {
  /* Matches the v1.4 manual's DA 0F 49 40 -> 3.1415925 example. */
  FORCE_FLOAT_ORDER_MANUAL_EXAMPLE = 0,
  /* Matches the v1.4 table's literal IEEE-754 big-endian wording. */
  FORCE_FLOAT_ORDER_BIG_ENDIAN
} ForceSensorFloatOrder;

typedef enum {
  FORCE_PARSE_NONE = 0,
  FORCE_PARSE_FRAME,
  FORCE_PARSE_ERROR
} ForceParseResult;

typedef struct {
  uint8_t state;
  uint8_t command;
  uint8_t payload[FORCE_SENSOR_MAX_PAYLOAD];
  uint8_t payload_length;
  uint8_t expected_payload_length;
} ForceSensorParser;

void force_sensor_parser_init(ForceSensorParser *parser);
ForceParseResult force_sensor_parser_feed(ForceSensorParser *parser,
                                          uint8_t byte,
                                          ForceSensorFrame *out);
size_t force_sensor_build_command(ForceSensorCommand command,
                                  const uint8_t *payload,
                                  uint8_t payload_length,
                                  uint8_t *out, size_t capacity);
size_t force_sensor_build_simple_command(ForceSensorCommand command,
                                         uint8_t out[5]);
size_t force_sensor_build_baud_command(ForceSensorBaudCode baud,
                                       uint8_t out[6]);
bool force_sensor_decode_sample(const ForceSensorFrame *frame,
                                ForceSensorSample *out);
bool force_sensor_decode_sample_order(const ForceSensorFrame *frame,
                                      ForceSensorFloatOrder order,
                                      ForceSensorSample *out);

#endif
