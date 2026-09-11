#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_bridge.h"
#include "app_config.h"
#include "board.h"
#include "byte_codec.h"
#include "robstride.h"
#include "udp_protocol.h"

enum {
  REPLY_OK = 0,
  REPLY_BAD_COMMAND = 2,
  REPLY_SAFETY_LOCKED = 3,
  REPLY_IO_ERROR = 5,
  REPLY_REPLAY = 6,
  REPLY_NOT_READY = 7
};

static uint8_t send_packet(uint8_t type, uint16_t flags, uint32_t sequence,
                           const uint8_t *payload, uint16_t payload_length,
                           uint32_t now_ms) {
  uint8_t datagram[UDP_PROTOCOL_HEADER_SIZE + UDP_PROTOCOL_MAX_PAYLOAD +
                   UDP_PROTOCOL_CRC_SIZE];
  uint8_t response[64];
  UdpPacketView reply;
  size_t datagram_length = udp_protocol_encode(
      type, flags, sequence, payload, payload_length, datagram,
      sizeof(datagram));
  size_t response_length = app_bridge_handle_datagram(
      datagram, datagram_length, now_ms, response, sizeof(response));
  assert(response_length > 0U);
  assert(udp_protocol_decode(response, response_length, &reply) ==
         UDP_DECODE_OK);
  assert(reply.payload_length == 4U);
  assert(reply.payload[0] == type);
  return reply.payload[1];
}

static uint8_t send_motor(const UdpMotorCommand *command, uint32_t sequence,
                          uint32_t now_ms) {
  uint8_t payload[28];
  assert(udp_protocol_encode_motor_command(command, payload) ==
         sizeof(payload));
  return send_packet(UDP_MSG_MOTOR_COMMAND, 0U, sequence, payload,
                     sizeof(payload), now_ms);
}

static void start_session(uint32_t sequence, uint32_t now_ms) {
  assert(send_packet(UDP_MSG_HELLO, 0U, sequence, NULL, 0U, now_ms) ==
         REPLY_OK);
}

static void inject_erob_status(uint16_t statusword, uint32_t now_ms) {
  CanFrame frame = {0};
  frame.id = 0x581U;
  frame.dlc = 8U;
  frame.data[0] = 0x4BU;
  codec_write_u16_le(&frame.data[1], 0x6041U);
  codec_write_u16_le(&frame.data[4], statusword);
  app_bridge_on_can_frame(APP_EROB_CAN_BUS, &frame, now_ms);
}

static void inject_robstride_feedback(uint8_t mode_state, uint8_t fault_bits,
                                      uint32_t now_ms) {
  CanFrame frame = {0};
  uint16_t data_field =
      (uint16_t)(1U | ((uint16_t)fault_bits << 8) |
                 ((uint16_t)(mode_state & 0x03U) << 14));
  frame.id = robstride_make_extended_id(
      ROBSTRIDE_COMM_FEEDBACK, data_field, APP_ROBSTRIDE_MASTER_ID);
  frame.dlc = 8U;
  frame.is_extended = true;
  codec_write_u16_be(&frame.data[0], 0x8000U);
  codec_write_u16_be(&frame.data[2], 0x8000U);
  codec_write_u16_be(&frame.data[4], 0x8000U);
  app_bridge_on_can_frame(APP_ROBSTRIDE_CAN_BUS, &frame, now_ms);
}

static UdpMotorCommand erob_command(uint8_t operation) {
  UdpMotorCommand command = {
      .can_bus = APP_EROB_CAN_BUS,
      .driver = UDP_DRIVER_EROB,
      .node_id = 1U,
      .operation = operation,
      .position = 1000.0F,
      .velocity = 100.0F,
      .torque = 0.0F,
      .kp = 0.0F,
      .kd = 0.0F,
      .timeout_ms = 200U,
      .options = APP_COMMAND_ARM_OPTION,
  };
  return command;
}

static UdpMotorCommand robstride_command(uint8_t operation,
                                         uint8_t model) {
  UdpMotorCommand command = {
      .can_bus = APP_ROBSTRIDE_CAN_BUS,
      .driver = UDP_DRIVER_ROBSTRIDE,
      .node_id = 1U,
      .operation = operation,
      .position = 0.0F,
      .velocity = 1.0F,
      .torque = 0.0F,
      .kp = 0.0F,
      .kd = 0.0F,
      .timeout_ms = 200U,
      .options = (uint16_t)(APP_COMMAND_ARM_OPTION | model),
  };
  return command;
}

static void test_erob_settle_gate(void) {
  UdpMotorCommand command;
  const CanFrame *frame;
  const CanFrame *last;

  mock_board_reset();
  app_bridge_init();
  start_session(1U, 100U);
  command = erob_command(UDP_MOTOR_ENABLE);
  assert(send_motor(&command, 2U, 100U) == REPLY_OK);
  assert(mock_board_can_success_count() == 5U);
  frame = mock_board_can_attempt(0U, NULL);
  assert(frame != NULL && codec_read_u16_le(&frame->data[1]) == 0x6040U);
  assert(codec_read_u16_le(&frame->data[4]) == 0x0006U);
  frame = mock_board_can_attempt(1U, NULL);
  assert(frame != NULL && codec_read_u16_le(&frame->data[1]) == 0x6060U);
  assert(frame->data[4] == 3U);
  frame = mock_board_can_attempt(2U, NULL);
  assert(frame != NULL && codec_read_u16_le(&frame->data[1]) == 0x60FFU);
  assert(codec_read_u32_le(&frame->data[4]) == 0U);
  frame = mock_board_can_attempt(3U, NULL);
  assert(frame != NULL && codec_read_u16_le(&frame->data[4]) == 0x0007U);
  frame = mock_board_can_attempt(4U, NULL);
  assert(frame != NULL && codec_read_u16_le(&frame->data[4]) == 0x000FU);

  command.operation = UDP_MOTOR_POSITION;
  assert(send_motor(&command, 3U, 599U) == REPLY_NOT_READY);
  assert(mock_board_can_success_count() == 6U);
  last = mock_board_can_attempt(mock_board_can_attempt_count() - 1U, NULL);
  assert(last != NULL && !last->is_extended && last->id == 0x601U);
  assert(last->data[0] == 0x2BU && codec_read_u16_le(&last->data[4]) == 2U);
}

static void test_erob_status_gate(void) {
  UdpMotorCommand command;

  mock_board_reset();
  app_bridge_init();
  start_session(4U, 100U);
  command = erob_command(UDP_MOTOR_ENABLE);
  assert(send_motor(&command, 5U, 100U) == REPLY_OK);
  inject_erob_status(0x0023U, 590U);
  command.operation = UDP_MOTOR_POSITION;
  assert(send_motor(&command, 6U, 600U) == REPLY_NOT_READY);
  assert(mock_board_can_success_count() == 6U);

  mock_board_reset();
  app_bridge_init();
  start_session(7U, 100U);
  command = erob_command(UDP_MOTOR_ENABLE);
  assert(send_motor(&command, 8U, 100U) == REPLY_OK);
  inject_erob_status(0x0027U, 349U);
  command.operation = UDP_MOTOR_POSITION;
  assert(send_motor(&command, 9U, 600U) == REPLY_NOT_READY);
  assert(mock_board_can_success_count() == 6U);
}

static void test_watchdog_stop_retry(void) {
  UdpMotorCommand command;
  size_t attempts;

  mock_board_reset();
  app_bridge_init();
  start_session(10U, 100U);
  command = erob_command(UDP_MOTOR_ENABLE);
  assert(send_motor(&command, 11U, 100U) == REPLY_OK);
  inject_erob_status(0x0027U, 590U);
  command.operation = UDP_MOTOR_POSITION;
  assert(send_motor(&command, 12U, 600U) == REPLY_OK);

  app_bridge_poll(799U);
  attempts = mock_board_can_attempt_count();
  mock_board_fail_next_can(1U);
  app_bridge_poll(800U);
  assert(mock_board_can_attempt_count() == attempts + 1U);
  app_bridge_poll(818U);
  assert(mock_board_can_attempt_count() == attempts + 1U);
  app_bridge_poll(820U);
  assert(mock_board_can_attempt_count() == attempts + 2U);
}

static void test_erob_position_trigger_and_reenable(void) {
  UdpMotorCommand command;
  const CanFrame *before_last;
  const CanFrame *last;
  size_t count;

  mock_board_reset();
  app_bridge_init();
  start_session(40U, 100U);
  command = erob_command(UDP_MOTOR_ENABLE);
  assert(send_motor(&command, 41U, 100U) == REPLY_OK);
  inject_erob_status(0x0027U, 590U);
  command.operation = UDP_MOTOR_POSITION;
  assert(send_motor(&command, 42U, 600U) == REPLY_OK);
  assert(mock_board_can_success_count() == 10U);
  count = mock_board_can_attempt_count();
  before_last = mock_board_can_attempt(count - 2U, NULL);
  last = mock_board_can_attempt(count - 1U, NULL);
  assert(before_last != NULL && last != NULL);
  assert(codec_read_u16_le(&before_last->data[1]) == 0x6040U);
  assert(codec_read_u16_le(&before_last->data[4]) == 0x000FU);
  assert(codec_read_u16_le(&last->data[1]) == 0x6040U);
  assert(codec_read_u16_le(&last->data[4]) == 0x001FU);

  mock_board_reset();
  app_bridge_init();
  start_session(50U, 100U);
  command = erob_command(UDP_MOTOR_ENABLE);
  assert(send_motor(&command, 51U, 100U) == REPLY_OK);
  assert(send_motor(&command, 52U, 400U) == REPLY_OK);
  app_bridge_poll(800U);
  last = mock_board_can_attempt(mock_board_can_attempt_count() - 1U, NULL);
  assert(last != NULL && last->data[0] == 0x40U);
  assert(codec_read_u16_le(&last->data[1]) == 0x6041U);
}

static void test_robstride_current_limit(void) {
  UdpMotorCommand command;
  const CanFrame *last;

  mock_board_reset();
  app_bridge_init();
  start_session(60U, 0U);
  command = robstride_command(UDP_MOTOR_ENABLE, 1U);
  assert(send_motor(&command, 61U, 1U) == REPLY_OK);
  inject_robstride_feedback(2U, 0U, 2U);
  command.operation = UDP_MOTOR_TORQUE;
  command.torque = 23.01F;
  assert(send_motor(&command, 62U, 3U) == REPLY_IO_ERROR);
  last = mock_board_can_attempt(mock_board_can_attempt_count() - 1U, NULL);
  assert(last != NULL && last->is_extended);
  assert(((last->id >> 24) & 0x1FU) == 4U);
}

static void test_robstride_feedback_gate(void) {
  UdpMotorCommand command;

  mock_board_reset();
  app_bridge_init();
  start_session(80U, 0U);
  command = robstride_command(UDP_MOTOR_ENABLE, 1U);
  assert(send_motor(&command, 81U, 1U) == REPLY_OK);
  command.operation = UDP_MOTOR_TORQUE;
  command.torque = 0.5F;
  assert(send_motor(&command, 82U, 2U) == REPLY_NOT_READY);
  assert(mock_board_can_success_count() == 5U);

  mock_board_reset();
  app_bridge_init();
  start_session(83U, 0U);
  command = robstride_command(UDP_MOTOR_ENABLE, 1U);
  assert(send_motor(&command, 84U, 1U) == REPLY_OK);
  inject_robstride_feedback(2U, 0U, 2U);
  command.operation = UDP_MOTOR_TORQUE;
  command.torque = 0.5F;
  assert(send_motor(&command, 85U, 3U) == REPLY_OK);
  assert(mock_board_can_success_count() == 6U);
}

static void test_robstride_safe_enable_sequence(void) {
  UdpMotorCommand command;
  const CanFrame *frame;

  mock_board_reset();
  app_bridge_init();
  start_session(70U, 0U);
  command = robstride_command(UDP_MOTOR_ENABLE, 1U);
  assert(send_motor(&command, 71U, 1U) == REPLY_OK);
  assert(mock_board_can_success_count() == 4U);
  assert(mock_board_delay_count() == 2U);
  assert(mock_board_delay_total_ms() == 2U);

  frame = mock_board_can_attempt(0U, NULL);
  assert(frame != NULL && ((frame->id >> 24) & 0x1FU) == 4U);
  frame = mock_board_can_attempt(1U, NULL);
  assert(frame != NULL && ((frame->id >> 24) & 0x1FU) == 18U);
  assert(codec_read_u16_le(&frame->data[0]) == ROBSTRIDE_PARAM_RUN_MODE);
  assert(frame->data[4] == ROBSTRIDE_RUN_CURRENT);
  frame = mock_board_can_attempt(2U, NULL);
  assert(frame != NULL && ((frame->id >> 24) & 0x1FU) == 18U);
  assert(codec_read_u16_le(&frame->data[0]) == ROBSTRIDE_PARAM_IQ_REF);
  assert(codec_read_u32_le(&frame->data[4]) == 0U);
  frame = mock_board_can_attempt(3U, NULL);
  assert(frame != NULL && ((frame->id >> 24) & 0x1FU) == 3U);
}

static void test_robstride_requires_local_enable(void) {
  UdpMotorCommand command;
  const CanFrame *last;

  mock_board_reset();
  app_bridge_init();
  start_session(90U, 0U);
  command = robstride_command(UDP_MOTOR_SET_ZERO, 1U);
  assert(send_motor(&command, 91U, 1U) == REPLY_OK);
  inject_robstride_feedback(2U, 0U, 2U);
  command.operation = UDP_MOTOR_TORQUE;
  command.torque = 0.5F;
  assert(send_motor(&command, 92U, 3U) == REPLY_NOT_READY);
  assert(mock_board_can_success_count() == 3U);
  last = mock_board_can_attempt(mock_board_can_attempt_count() - 1U, NULL);
  assert(last != NULL && ((last->id >> 24) & 0x1FU) == 4U);
}

static void test_sequence_and_validation(void) {
  UdpMotorCommand command;

  mock_board_reset();
  app_bridge_init();
  start_session(20U, 0U);
  command = erob_command(UDP_MOTOR_ENABLE);
  assert(send_motor(&command, 21U, 0U) == REPLY_OK);
  assert(send_motor(&command, 21U, 1U) == REPLY_REPLAY);
  assert(send_packet(UDP_MSG_HELLO, 0U, 1U, NULL, 0U, 2U) == REPLY_OK);
  assert(send_packet(UDP_MSG_SAFE_STOP, 1U, 2U, NULL, 0U, 3U) ==
         REPLY_BAD_COMMAND);
  command.options = 0U;
  assert(send_motor(&command, 2U, 4U) == REPLY_SAFETY_LOCKED);
  command.options = APP_COMMAND_ARM_OPTION;
  command.node_id = 0U;
  assert(send_motor(&command, 3U, 5U) == REPLY_BAD_COMMAND);
  command = robstride_command(UDP_MOTOR_STOP, 7U);
  assert(send_motor(&command, 4U, 6U) == REPLY_BAD_COMMAND);
}

static void test_robstride_stop_model_match(void) {
  UdpMotorCommand command;

  mock_board_reset();
  app_bridge_init();
  start_session(100U, 0U);
  command = robstride_command(UDP_MOTOR_ENABLE, 1U);
  assert(send_motor(&command, 101U, 1U) == REPLY_OK);

  command = robstride_command(UDP_MOTOR_STOP, 2U);
  assert(send_motor(&command, 102U, 2U) == REPLY_BAD_COMMAND);
  command = robstride_command(UDP_MOTOR_STOP, 1U);
  assert(send_motor(&command, 103U, 3U) == REPLY_OK);

  app_bridge_on_can_frame(APP_ROBSTRIDE_CAN_BUS, NULL, 4U);
}

static void test_force_dma_to_telemetry(void) {
  static const uint8_t partial[] = {0xAA, 0x55, 0x03, 0x00};
  static const uint8_t sample[] = {
      0xAA, 0x55, 0x03,
      0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x40,
      0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x80, 0x40,
      0x00, 0x00, 0xA0, 0x40, 0x00, 0x00, 0xC0, 0x40,
      0x0D, 0x0A};
  uint8_t telemetry[68];
  uint8_t force_command = 0x03U;
  const uint8_t *uart_data;

  mock_board_reset();
  app_bridge_init();
  assert(mock_board_inject_force(partial, sizeof(partial)));
  app_bridge_poll(121U);
  mock_board_fault_force_rx();
  app_bridge_poll(122U);
  assert(mock_board_inject_force(sample, sizeof(sample)));
  app_bridge_poll(123U);
  assert(app_bridge_build_telemetry(124U, telemetry, sizeof(telemetry)) ==
         sizeof(telemetry));
  assert((codec_read_u32_le(&telemetry[4]) & 1U) != 0U);
  assert((codec_read_u32_le(&telemetry[4]) & (1U << 3)) != 0U);
  assert(codec_read_u32_le(&telemetry[8]) == 123U);
  assert(fabsf(codec_read_f32_le(&telemetry[12]) - 1.0F) < 0.0001F);
  assert(fabsf(codec_read_f32_le(&telemetry[32]) - 6.0F) < 0.0001F);

  start_session(30U, 124U);
  assert(send_packet(UDP_MSG_FORCE_COMMAND, 0U, 31U, &force_command, 1U,
                     125U) == REPLY_OK);
  assert(mock_board_uart_tx(&uart_data) == 5U);
  assert(memcmp(uart_data, "\xAA\x55\x03\x0D\x0A", 5U) == 0);

  app_bridge_poll(1124U);
  assert(app_bridge_build_telemetry(1125U, telemetry, sizeof(telemetry)) ==
         sizeof(telemetry));
  assert((codec_read_u32_le(&telemetry[4]) & 1U) == 0U);
}

int main(void) {
  test_erob_settle_gate();
  test_erob_status_gate();
  test_watchdog_stop_retry();
  test_erob_position_trigger_and_reenable();
  test_robstride_current_limit();
  test_robstride_feedback_gate();
  test_robstride_safe_enable_sequence();
  test_robstride_requires_local_enable();
  test_sequence_and_validation();
  test_robstride_stop_model_match();
  test_force_dma_to_telemetry();
  puts("application bridge tests passed");
  return 0;
}
