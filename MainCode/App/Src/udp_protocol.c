#include "udp_protocol.h"

#include <math.h>

#include "byte_codec.h"

/** @brief 计算主 UDP 协议帧使用的 CRC32。 */
uint32_t udp_protocol_crc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFU;
  size_t i;
  unsigned bit;

  if (data == NULL && length != 0U) {
    return 0U;
  }
  for (i = 0U; i < length; ++i) {
    crc ^= data[i];
    for (bit = 0U; bit < 8U; ++bit) {
      uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
      crc = (crc >> 1) ^ (0xEDB88320U & mask);
    }
  }
  return ~crc;
}

/** @brief 编码带协议头和 CRC32 的 UDP 数据报。 */
size_t udp_protocol_encode(uint8_t type, uint16_t flags, uint32_t sequence,
                           const uint8_t *payload, uint16_t payload_length,
                           uint8_t *out, size_t capacity) {
  size_t total = UDP_PROTOCOL_HEADER_SIZE + payload_length +
                 UDP_PROTOCOL_CRC_SIZE;
  uint32_t crc;

  if (out == NULL || payload_length > UDP_PROTOCOL_MAX_PAYLOAD ||
      capacity < total || (payload_length != 0U && payload == NULL)) {
    return 0U;
  }

  out[0] = UDP_PROTOCOL_MAGIC_0;
  out[1] = UDP_PROTOCOL_MAGIC_1;
  out[2] = UDP_PROTOCOL_VERSION;
  out[3] = type;
  codec_write_u16_le(&out[4], flags);
  codec_write_u16_le(&out[6], payload_length);
  codec_write_u32_le(&out[8], sequence);
  for (uint16_t i = 0U; i < payload_length; ++i) {
    out[UDP_PROTOCOL_HEADER_SIZE + i] = payload[i];
  }
  crc = udp_protocol_crc32(out, UDP_PROTOCOL_HEADER_SIZE + payload_length);
  codec_write_u32_le(&out[UDP_PROTOCOL_HEADER_SIZE + payload_length], crc);
  return total;
}

/** @brief 校验并解析 UDP 数据报，返回载荷视图。 */
UdpDecodeStatus udp_protocol_decode(const uint8_t *datagram, size_t length,
                                    UdpPacketView *out) {
  uint16_t payload_length;
  size_t expected;
  uint32_t received_crc;
  uint32_t computed_crc;

  if (datagram == NULL || out == NULL ||
      length < UDP_PROTOCOL_HEADER_SIZE + UDP_PROTOCOL_CRC_SIZE) {
    return UDP_DECODE_TOO_SHORT;
  }
  if (datagram[0] != UDP_PROTOCOL_MAGIC_0 ||
      datagram[1] != UDP_PROTOCOL_MAGIC_1) {
    return UDP_DECODE_MAGIC;
  }
  if (datagram[2] != UDP_PROTOCOL_VERSION) {
    return UDP_DECODE_VERSION;
  }
  payload_length = codec_read_u16_le(&datagram[6]);
  expected = UDP_PROTOCOL_HEADER_SIZE + payload_length + UDP_PROTOCOL_CRC_SIZE;
  if (payload_length > UDP_PROTOCOL_MAX_PAYLOAD || length != expected) {
    return UDP_DECODE_LENGTH;
  }
  received_crc = codec_read_u32_le(&datagram[length - UDP_PROTOCOL_CRC_SIZE]);
  computed_crc = udp_protocol_crc32(datagram, length - UDP_PROTOCOL_CRC_SIZE);
  if (received_crc != computed_crc) {
    return UDP_DECODE_CRC;
  }

  out->type = datagram[3];
  out->flags = codec_read_u16_le(&datagram[4]);
  out->sequence = codec_read_u32_le(&datagram[8]);
  out->payload = &datagram[UDP_PROTOCOL_HEADER_SIZE];
  out->payload_length = payload_length;
  return UDP_DECODE_OK;
}

bool udp_protocol_decode_motor_command(const UdpPacketView *packet,
                                       UdpMotorCommand *out) {
  if (packet == NULL || out == NULL || packet->type != UDP_MSG_MOTOR_COMMAND ||
      packet->payload == NULL || packet->payload_length != 28U) {
    return false;
  }
  out->can_bus = packet->payload[0];
  out->driver = packet->payload[1];
  out->node_id = packet->payload[2];
  out->operation = packet->payload[3];
  out->position = codec_read_f32_le(&packet->payload[4]);
  out->velocity = codec_read_f32_le(&packet->payload[8]);
  out->torque = codec_read_f32_le(&packet->payload[12]);
  out->kp = codec_read_f32_le(&packet->payload[16]);
  out->kd = codec_read_f32_le(&packet->payload[20]);
  out->timeout_ms = codec_read_u16_le(&packet->payload[24]);
  out->options = codec_read_u16_le(&packet->payload[26]);

  return (out->can_bus == 1U || out->can_bus == 2U) &&
         (out->driver == UDP_DRIVER_EROB ||
          out->driver == UDP_DRIVER_ROBSTRIDE) &&
         out->node_id >= 1U && out->node_id <= 127U &&
         out->operation <= UDP_MOTOR_FAULT_RESET &&
         isfinite(out->position) && isfinite(out->velocity) &&
         isfinite(out->torque) && isfinite(out->kp) && isfinite(out->kd) &&
         out->timeout_ms >= 20U && out->timeout_ms <= 5000U;
}

/** @brief 将电机命令结构编码为 28 字节 UDP 负载。 */
size_t udp_protocol_encode_motor_command(const UdpMotorCommand *command,
                                         uint8_t payload[28]) {
  if (command == NULL || payload == NULL) {
    return 0U;
  }
  payload[0] = command->can_bus;
  payload[1] = command->driver;
  payload[2] = command->node_id;
  payload[3] = command->operation;
  codec_write_f32_le(&payload[4], command->position);
  codec_write_f32_le(&payload[8], command->velocity);
  codec_write_f32_le(&payload[12], command->torque);
  codec_write_f32_le(&payload[16], command->kp);
  codec_write_f32_le(&payload[20], command->kd);
  codec_write_u16_le(&payload[24], command->timeout_ms);
  codec_write_u16_le(&payload[26], command->options);
  return 28U;
}
