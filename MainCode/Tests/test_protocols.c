#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "arm_kinematics.h"
#include "arm_nuc_protocol.h"
#include "erob_canopen.h"
#include "force_sensor.h"
#include "legacy_can_bridge.h"
#include "legacy_udp.h"
#include "robstride.h"
#include "udp_protocol.h"

static int close_enough(float a, float b, float tolerance) {
  return fabsf(a - b) <= tolerance;
}

static ArmKinematicsConfig arm_test_config(void) {
  ArmKinematicsConfig config = {0};
  arm_kinematics_config_default(&config);
  config.position_tolerance_mm = 0.20F;
  config.max_iterations = 80U;
  return config;
}

static void test_arm_kinematics(void) {
  ArmKinematicsConfig config = arm_test_config();
  const float source[ARM_JOINT_COUNT] = {0.40F, 0.20F, -0.30F, 0.25F,
                                         -0.15F};
  float seed[ARM_JOINT_COUNT] = {0.0F};
  float solved[ARM_JOINT_COUNT] = {0.0F};
  ArmPoint target;
  ArmPoint reached;
  ArmIkResult result;

  assert(arm_kinematics_forward(&config, source, &target));
  assert(arm_kinematics_inverse(&config, &target, seed, solved, &result) ==
         ARM_IK_OK);
  assert(result.position_error_mm <= config.position_tolerance_mm);
  assert(result.orientation_error_rad <= config.orientation_tolerance_rad);
  assert(arm_kinematics_forward(&config, solved, &reached));
  assert(close_enough(reached.x_mm, target.x_mm, 0.25F));
  assert(close_enough(reached.y_mm, target.y_mm, 0.25F));
  assert(close_enough(reached.z_mm, target.z_mm, 0.25F));
  assert(close_enough(solved[0], source[0], 0.001F));

  target.x_mm = 2000.0F;
  target.y_mm = 0.0F;
  target.z_mm = 0.0F;
  assert(arm_kinematics_inverse(&config, &target, seed, solved, &result) ==
         ARM_IK_UNREACHABLE);
}

static uint32_t next_random(uint32_t *state) {
  uint32_t value = *state;
  value ^= value << 13;
  value ^= value >> 17;
  value ^= value << 5;
  *state = value;
  return value;
}

static void test_erob(void) {
  CanFrame frame;
  ErobSdoResult result;

  assert(erob_make_target_position(1U, 0x12345678, &frame));
  assert(frame.id == 0x601U && !frame.is_extended && frame.dlc == 8U);
  const uint8_t expected[] = {0x23, 0x7A, 0x60, 0x00,
                              0x78, 0x56, 0x34, 0x12};
  assert(memcmp(frame.data, expected, sizeof(expected)) == 0);

  memset(&frame, 0, sizeof(frame));
  frame.id = 0x581U;
  frame.dlc = 8U;
  frame.data[0] = 0x4BU;
  frame.data[1] = 0x41U;
  frame.data[2] = 0x60U;
  frame.data[4] = 0x27U;
  assert(erob_parse_sdo_response(&frame, &result));
  assert(result.kind == EROB_SDO_UPLOAD_OK && result.value_size == 2U);
  assert(erob_ds402_state((uint16_t)result.value) ==
         EROB_DS402_OPERATION_ENABLED);
  assert(erob_ds402_next_controlword(EROB_DS402_SWITCH_ON_DISABLED) == 6U);
  assert(erob_ds402_next_controlword(EROB_DS402_FAULT) == 0x80U);
  assert(!erob_make_nmt((ErobNmtCommand)0x7FU, 1U, &frame));
  assert(!erob_make_mode(1U, (ErobOperationMode)2, &frame));
  frame.is_remote = true;
  assert(!erob_parse_sdo_response(&frame, &result));
}

static void test_robstride(void) {
  RobStrideMotionCommand command = {0};
  RobStrideFeedback feedback;
  CanFrame frame;

  assert(robstride_make_motion(ROBSTRIDE_RS01, 1U, &command, &frame));
  assert(frame.is_extended && frame.dlc == 8U);
  assert(frame.id == 0x01800001U);
  const uint8_t expected[] = {0x80, 0x00, 0x80, 0x00,
                              0x00, 0x00, 0x00, 0x00};
  assert(memcmp(frame.data, expected, sizeof(expected)) == 0);

  memset(&frame, 0, sizeof(frame));
  frame.id = 0x028301FDU;
  frame.is_extended = true;
  frame.dlc = 8U;
  frame.data[0] = 0x80U;
  frame.data[1] = 0x00U;
  frame.data[2] = 0xFFU;
  frame.data[3] = 0xFFU;
  frame.data[4] = 0x80U;
  frame.data[5] = 0x00U;
  frame.data[6] = 0x00U;
  frame.data[7] = 0xFDU;
  assert(robstride_parse_feedback(ROBSTRIDE_RS01,
                                  ROBSTRIDE_MASTER_ID_DEFAULT,
                                  &frame, &feedback));
  assert(feedback.motor_id == 1U && feedback.mode_state == 2U);
  assert(feedback.fault_bits == 3U);
  assert(close_enough(feedback.position_rad, 0.0F, 0.001F));
  assert(close_enough(feedback.velocity_rad_s, 44.0F, 0.001F));
  assert(close_enough(feedback.torque_nm, 0.0F, 0.001F));
  assert(close_enough(feedback.temperature_c, 25.3F, 0.01F));
  frame.is_remote = true;
  assert(!robstride_parse_feedback(ROBSTRIDE_RS01,
                                   ROBSTRIDE_MASTER_ID_DEFAULT,
                                   &frame, &feedback));

  command.position_rad = ROBSTRIDE_POSITION_LIMIT_RAD + 0.01F;
  assert(!robstride_make_motion(ROBSTRIDE_RS01, 1U, &command, &frame));
  command.position_rad = 0.0F;
  command.kp = -0.01F;
  assert(!robstride_make_motion(ROBSTRIDE_RS01, 1U, &command, &frame));
  command.kp = 0.0F;
  assert(!robstride_make_motion(ROBSTRIDE_RS01, 128U, &command, &frame));
  assert(!robstride_make_enable(128U, ROBSTRIDE_MASTER_ID_DEFAULT, &frame));

  assert(close_enough(robstride_limits(ROBSTRIDE_RS00)->current_a, 16.0F,
                      0.001F));
  assert(close_enough(robstride_limits(ROBSTRIDE_RS01)->current_a, 23.0F,
                      0.001F));
  assert(close_enough(robstride_limits(ROBSTRIDE_RS02)->current_a, 16.0F,
                      0.001F));
  assert(close_enough(robstride_limits(ROBSTRIDE_RS03)->current_a, 43.0F,
                      0.001F));
  assert(close_enough(robstride_limits(ROBSTRIDE_RS04)->current_a, 90.0F,
                      0.001F));
  assert(close_enough(robstride_limits(ROBSTRIDE_RS05)->current_a, 11.0F,
                      0.001F));
  assert(close_enough(robstride_limits(ROBSTRIDE_RS06)->current_a, 57.0F,
                      0.001F));

  assert(robstride_make_discover(ROBSTRIDE_MASTER_ID_DEFAULT, &frame));
  assert(frame.id == 0x0000FD7FU && frame.is_extended);
  assert(robstride_make_set_can_id(0x7FU, 1U,
                                   ROBSTRIDE_MASTER_ID_DEFAULT, &frame));
  assert(frame.id == 0x0701FD7FU);
  assert(robstride_make_set_baud(1U, ROBSTRIDE_MASTER_ID_DEFAULT,
                                 ROBSTRIDE_BAUD_500K, &frame));
  const uint8_t baud_expected[] = {1, 2, 3, 4, 5, 6, 2, 0};
  assert(memcmp(frame.data, baud_expected, sizeof(baud_expected)) == 0);
  assert(!robstride_make_set_protocol(
      1U, ROBSTRIDE_MASTER_ID_DEFAULT, (RobStrideProtocol)-1, &frame));

  RobStrideParameterReply parameter;
  memset(&frame, 0, sizeof(frame));
  frame.id = 0x110001FDU;
  frame.is_extended = true;
  frame.dlc = 8U;
  frame.data[0] = 0x1EU;
  frame.data[1] = 0x70U;
  frame.data[6] = 0xF0U;
  frame.data[7] = 0x41U;
  assert(robstride_parse_parameter_reply(ROBSTRIDE_MASTER_ID_DEFAULT,
                                         &frame, &parameter));
  assert(parameter.motor_id == 1U && parameter.success);
  assert(parameter.index == 0x701EU);
  assert(close_enough(parameter.float_value, 30.0F, 0.001F));
  frame.id = 0x110001FEU;
  assert(!robstride_parse_parameter_reply(ROBSTRIDE_MASTER_ID_DEFAULT,
                                          &frame, &parameter));
}

static void test_force_sensor(void) {
  static const uint8_t sample[] = {
      0xAA, 0x55, 0x02,
      0xDA, 0x0F, 0x49, 0x40, 0xDA, 0x0F, 0x49, 0xC0,
      0xDA, 0x0F, 0x49, 0x40, 0xDA, 0x0F, 0x49, 0xC0,
      0xDA, 0x0F, 0x49, 0x40, 0xDA, 0x0F, 0x49, 0xC0,
      0x0D, 0x0A};
  ForceSensorParser parser;
  ForceSensorFrame frame;
  ForceSensorSample decoded;
  ForceParseResult status = FORCE_PARSE_NONE;

  force_sensor_parser_init(&parser);
  assert(force_sensor_parser_feed(&parser, 0x12U, &frame) == FORCE_PARSE_NONE);
  for (size_t i = 0; i < sizeof(sample); ++i) {
    status = force_sensor_parser_feed(&parser, sample[i], &frame);
  }
  assert(status == FORCE_PARSE_FRAME && frame.payload_length == 24U);
  assert(force_sensor_decode_sample(&frame, &decoded));
  assert(close_enough(decoded.fx, 3.1415925F, 0.000001F));
  assert(close_enough(decoded.fy, -3.1415925F, 0.000001F));
  assert(close_enough(decoded.mz, -3.1415925F, 0.000001F));
  assert(!force_sensor_decode_sample_order(
      &frame, (ForceSensorFloatOrder)2, &decoded));

  uint8_t command[6];
  assert(force_sensor_build_baud_command(FORCE_BAUD_921600, command) == 6U);
  const uint8_t baud_expected[] = {0xAA, 0x55, 0x04, 0x03, 0x0D, 0x0A};
  assert(memcmp(command, baud_expected, sizeof(command)) == 0);
  uint8_t oversized_payload[FORCE_SENSOR_MAX_PAYLOAD + 1U] = {0};
  uint8_t oversized_frame[FORCE_SENSOR_MAX_PAYLOAD + 6U];
  assert(force_sensor_build_command(FORCE_CMD_ENTER_DEBUG,
                                    oversized_payload,
                                    (uint8_t)sizeof(oversized_payload),
                                    oversized_frame,
                                    sizeof(oversized_frame)) == 0U);

  static const uint8_t baud_reply[] = {
      0xAA, 0x55, 0x04, 'R', 'e', 's', 't', 'a', 'r', 't', ' ',
      'p',  'l',  'e',  'a', 's', 'e', '!', 0x0D, 0x0A};
  force_sensor_parser_init(&parser);
  for (size_t i = 0; i < sizeof(baud_reply); ++i) {
    status = force_sensor_parser_feed(&parser, baud_reply[i], &frame);
  }
  assert(status == FORCE_PARSE_FRAME && frame.command == FORCE_CMD_SET_BAUD);
  assert(frame.payload_length == sizeof("Restart please!") - 1U);
}

static void test_udp(void) {
  static const uint8_t check[] = "123456789";
  uint8_t payload[28];
  uint8_t datagram[64];
  UdpMotorCommand command = {
      .can_bus = 2,
      .driver = UDP_DRIVER_ROBSTRIDE,
      .node_id = 1,
      .operation = UDP_MOTOR_IMPEDANCE,
      .position = 1.25F,
      .velocity = -2.5F,
      .torque = 0.75F,
      .kp = 10.0F,
      .kd = 0.4F,
      .timeout_ms = 200,
      .options = ROBSTRIDE_RS01,
  };
  UdpMotorCommand decoded;
  UdpPacketView packet;

  assert(udp_protocol_crc32(check, 9U) == 0xCBF43926U);
  assert(udp_protocol_encode_motor_command(&command, payload) == 28U);
  size_t length = udp_protocol_encode(UDP_MSG_MOTOR_COMMAND, 0U, 42U,
                                      payload, sizeof(payload), datagram,
                                      sizeof(datagram));
  assert(length == 44U);
  assert(udp_protocol_decode(datagram, length, &packet) == UDP_DECODE_OK);
  assert(packet.sequence == 42U);
  assert(udp_protocol_decode_motor_command(&packet, &decoded));
  assert(decoded.node_id == 1U && decoded.timeout_ms == 200U);
  assert(close_enough(decoded.position, 1.25F, 0.00001F));
  command.node_id = 128U;
  assert(udp_protocol_encode_motor_command(&command, payload) == 28U);
  length = udp_protocol_encode(UDP_MSG_MOTOR_COMMAND, 0U, 43U,
                               payload, sizeof(payload), datagram,
                               sizeof(datagram));
  assert(udp_protocol_decode(datagram, length, &packet) == UDP_DECODE_OK);
  assert(!udp_protocol_decode_motor_command(&packet, &decoded));
  datagram[12] ^= 1U;
  assert(udp_protocol_decode(datagram, length, &packet) == UDP_DECODE_CRC);
}

static void test_legacy_udp(void) {
  static const uint8_t input[] = "SQ_1000,L_-1500_1500,D_1,ZT_1600/";
  LegacyUdpCommand command;
  assert(legacy_udp_parse(input, sizeof(input) - 1U, &command));
  assert(!command.stop_requested && command.has_track && command.has_light);
  assert(command.left_track == -1500 && command.right_track == 1500);
  assert(command.light_on == 1U);
  static const uint8_t comma_track[] = "L_1500,1000,D:0/";
  assert(legacy_udp_parse(comma_track, sizeof(comma_track) - 1U, &command));
  assert(command.has_track && command.left_track == 1500 &&
         command.right_track == 1000 && command.has_light &&
         command.light_on == 0U);
  static const uint8_t old_zt_stop[] = "ZT/";
  assert(legacy_udp_parse(old_zt_stop, sizeof(old_zt_stop) - 1U, &command));
  assert(command.stop_requested);
  static const uint8_t old_stop[] = "ST:P/";
  assert(legacy_udp_parse(old_stop, sizeof(old_stop) - 1U, &command));
  assert(command.stop_requested);
  static const uint8_t embedded_stop[] = "XST:P/";
  assert(!legacy_udp_parse(embedded_stop, sizeof(embedded_stop) - 1U,
                           &command));
  static const uint8_t drill_only[] = "ZT_1000/";
  assert(!legacy_udp_parse(drill_only, sizeof(drill_only) - 1U, &command));
  static const uint8_t malformed_number[] =
      "L_999999999999999999999999999999999999999999999999999999/";
  assert(!legacy_udp_parse(malformed_number,
                           sizeof(malformed_number) - 1U, &command));
}

static void test_arm_nuc_protocol(void) {
  ArmNucCommand command;
  static const uint8_t position[] = "PXYZ:+030000,-001250,+012500,/";
  static const uint8_t gripper[] = "JZH_+500_1000/";
  static const uint8_t bundle[] =
      "PXYZ:+030000,+000000,+012500,SQ_1000,JZH_+500_1000,/";
  static const uint8_t stop[] = "ST:P/";

  assert(arm_nuc_parse(position, sizeof(position) - 1U, &command));
  assert(command.type == ARM_NUC_POSITION && command.x_centi_mm == 30000 &&
         command.y_centi_mm == -1250 && command.z_centi_mm == 12500 &&
         command.has_position && !command.has_gripper);
  assert(arm_nuc_parse(gripper, sizeof(gripper) - 1U, &command));
  assert(command.type == ARM_NUC_GRIPPER && command.gripper_millirad == 500 &&
         command.gripper_current_ma == 1000U && !command.has_position &&
         command.has_gripper);
  assert(arm_nuc_parse(bundle, sizeof(bundle) - 1U, &command));
  assert(command.type == ARM_NUC_BUNDLE && command.has_position &&
         command.has_gripper);
  assert(arm_nuc_parse(stop, sizeof(stop) - 1U, &command));
  assert(command.type == ARM_NUC_STOP);
  assert(!arm_nuc_parse((const uint8_t *)"PXYZ:1,2,3/", 11U, &command));
}

static void test_legacy_can_bridge(void) {
  const uint8_t datagram[LEGACY_CAN_BRIDGE_FRAME_SIZE] = {
      4, 0, 0x04, 0x00, 0xFD, 0x01, 8,
      0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t encoded[LEGACY_CAN_BRIDGE_FRAME_SIZE];
  CanFrame frame;
  assert(legacy_can_bridge_decode(datagram, sizeof(datagram), &frame));
  assert(frame.id == 0x0400FD01U && frame.is_extended);
  assert(legacy_can_bridge_is_safe_stop(&frame));
  assert(legacy_can_bridge_encode(&frame, encoded) == sizeof(encoded));
  assert(memcmp(datagram, encoded, sizeof(datagram)) == 0);
  frame.data[1] = 0xC4U;
  assert(!legacy_can_bridge_is_safe_stop(&frame));
  frame.data[1] = 0U;
  frame.is_remote = true;
  assert(!legacy_can_bridge_is_safe_stop(&frame));
}

static void test_parser_robustness(void) {
  uint8_t bytes[272];
  uint32_t random_state = 0x51A7C0DEU;
  ForceSensorParser force_parser;
  ForceSensorFrame force_frame;
  UdpPacketView packet;
  LegacyUdpCommand legacy;
  CanFrame can_frame;

  force_sensor_parser_init(&force_parser);
  for (size_t iteration = 0U; iteration < 50000U; ++iteration) {
    uint8_t byte = (uint8_t)next_random(&random_state);
    ForceParseResult result =
        force_sensor_parser_feed(&force_parser, byte, &force_frame);
    assert(result <= FORCE_PARSE_ERROR);
  }

  for (size_t iteration = 0U; iteration < 5000U; ++iteration) {
    size_t length = next_random(&random_state) % (sizeof(bytes) + 1U);
    for (size_t i = 0U; i < length; ++i) {
      bytes[i] = (uint8_t)next_random(&random_state);
    }
    (void)udp_protocol_decode(bytes, length, &packet);
    if (length > 0U && length < 192U) {
      (void)legacy_udp_parse(bytes, length, &legacy);
    }
    if (length == LEGACY_CAN_BRIDGE_FRAME_SIZE) {
      (void)legacy_can_bridge_decode(bytes, length, &can_frame);
    }
  }
}

int main(void) {
  test_arm_kinematics();
  test_erob();
  test_robstride();
  test_force_sensor();
  test_udp();
  test_legacy_udp();
  test_arm_nuc_protocol();
  test_legacy_can_bridge();
  test_parser_robustness();
  puts("protocol tests passed");
  return 0;
}
