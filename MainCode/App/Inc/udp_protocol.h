#ifndef UDP_PROTOCOL_H
#define UDP_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UDP_PROTOCOL_MAGIC_0 0x55U
#define UDP_PROTOCOL_MAGIC_1 0x57U
#define UDP_PROTOCOL_VERSION 1U
#define UDP_PROTOCOL_HEADER_SIZE 12U
#define UDP_PROTOCOL_CRC_SIZE 4U
#define UDP_PROTOCOL_MAX_PAYLOAD 256U

typedef enum {
  UDP_MSG_HELLO = 0x01,
  UDP_MSG_MOTOR_COMMAND = 0x10,
  UDP_MSG_SAFE_STOP = 0x11,
  UDP_MSG_FORCE_COMMAND = 0x20,
  UDP_MSG_TELEMETRY = 0x80,
  UDP_MSG_ACK = 0x81,
  UDP_MSG_ERROR = 0x82,
  UDP_MSG_ARM_STATUS = 0x83
} UdpMessageType;

typedef enum {
  UDP_DECODE_OK = 0,
  UDP_DECODE_TOO_SHORT,
  UDP_DECODE_MAGIC,
  UDP_DECODE_VERSION,
  UDP_DECODE_LENGTH,
  UDP_DECODE_CRC
} UdpDecodeStatus;

typedef struct {
  uint8_t type;
  uint16_t flags;
  uint32_t sequence;
  const uint8_t *payload;
  uint16_t payload_length;
} UdpPacketView;

typedef enum {
  UDP_MOTOR_DISABLE = 0,
  UDP_MOTOR_ENABLE = 1,
  UDP_MOTOR_STOP = 2,
  UDP_MOTOR_SET_ZERO = 3,
  UDP_MOTOR_POSITION = 4,
  UDP_MOTOR_VELOCITY = 5,
  UDP_MOTOR_TORQUE = 6,
  UDP_MOTOR_IMPEDANCE = 7,
  UDP_MOTOR_FAULT_RESET = 8
} UdpMotorOperation;

typedef enum {
  UDP_DRIVER_EROB = 1,
  UDP_DRIVER_ROBSTRIDE = 2
} UdpMotorDriver;

typedef struct {
  uint8_t can_bus;
  uint8_t driver;
  uint8_t node_id;
  uint8_t operation;
  float position;
  float velocity;
  float torque;
  float kp;
  float kd;
  uint16_t timeout_ms;
  uint16_t options;
} UdpMotorCommand;

uint32_t udp_protocol_crc32(const uint8_t *data, size_t length);
size_t udp_protocol_encode(uint8_t type, uint16_t flags, uint32_t sequence,
                           const uint8_t *payload, uint16_t payload_length,
                           uint8_t *out, size_t capacity);
UdpDecodeStatus udp_protocol_decode(const uint8_t *datagram, size_t length,
                                    UdpPacketView *out);
bool udp_protocol_decode_motor_command(const UdpPacketView *packet,
                                       UdpMotorCommand *out);
size_t udp_protocol_encode_motor_command(const UdpMotorCommand *command,
                                         uint8_t payload[28]);

#endif
