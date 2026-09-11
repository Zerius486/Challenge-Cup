#ifndef ROBSTRIDE_H
#define ROBSTRIDE_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

#define ROBSTRIDE_MASTER_ID_DEFAULT 0xFDU
#define ROBSTRIDE_POSITION_LIMIT_RAD 12.57F

typedef enum {
  ROBSTRIDE_RS00 = 0,
  ROBSTRIDE_RS01,
  ROBSTRIDE_RS02,
  ROBSTRIDE_RS03,
  ROBSTRIDE_RS04,
  ROBSTRIDE_RS05,
  ROBSTRIDE_RS06,
  ROBSTRIDE_MODEL_COUNT
} RobStrideModel;

typedef struct {
  float velocity_rad_s;
  float torque_nm;
  float current_a;
  float kp;
  float kd;
} RobStrideLimits;

typedef enum {
  ROBSTRIDE_COMM_DISCOVER = 0x00,
  ROBSTRIDE_COMM_MOTION = 0x01,
  ROBSTRIDE_COMM_FEEDBACK = 0x02,
  ROBSTRIDE_COMM_ENABLE = 0x03,
  ROBSTRIDE_COMM_STOP = 0x04,
  ROBSTRIDE_COMM_SET_ZERO = 0x06,
  ROBSTRIDE_COMM_SET_CAN_ID = 0x07,
  ROBSTRIDE_COMM_READ_PARAM = 0x11,
  ROBSTRIDE_COMM_WRITE_PARAM = 0x12,
  ROBSTRIDE_COMM_FAULT = 0x15,
  ROBSTRIDE_COMM_SAVE = 0x16,
  ROBSTRIDE_COMM_SET_BAUD = 0x17,
  ROBSTRIDE_COMM_REPORT = 0x18,
  ROBSTRIDE_COMM_PROTOCOL = 0x19
} RobStrideCommType;

typedef enum {
  ROBSTRIDE_RUN_MOTION = 0,
  ROBSTRIDE_RUN_POSITION = 1,
  ROBSTRIDE_RUN_VELOCITY = 2,
  ROBSTRIDE_RUN_CURRENT = 3,
  ROBSTRIDE_RUN_CSP = 5
} RobStrideRunMode;

typedef struct {
  float position_rad;
  float velocity_rad_s;
  float torque_nm;
  float kp;
  float kd;
} RobStrideMotionCommand;

typedef struct {
  uint8_t motor_id;
  uint8_t mode_state;
  uint8_t fault_bits;
  float position_rad;
  float velocity_rad_s;
  float torque_nm;
  float temperature_c;
} RobStrideFeedback;

typedef struct {
  RobStrideCommType type;
  uint8_t motor_id;
  bool success;
  uint16_t index;
  uint32_t raw_value;
  float float_value;
} RobStrideParameterReply;

typedef enum {
  ROBSTRIDE_BAUD_1M = 1,
  ROBSTRIDE_BAUD_500K = 2,
  ROBSTRIDE_BAUD_250K = 3,
  ROBSTRIDE_BAUD_125K = 4
} RobStrideBaud;

typedef enum {
  ROBSTRIDE_PROTOCOL_PRIVATE = 0,
  ROBSTRIDE_PROTOCOL_CANOPEN = 1,
  ROBSTRIDE_PROTOCOL_MIT = 2
} RobStrideProtocol;

typedef struct {
  uint8_t motor_id;
  uint8_t uid[8];
} RobStrideDiscoveryReply;

typedef struct {
  uint8_t motor_id;
  uint32_t faults;
  uint32_t warnings;
} RobStrideFaultReply;

typedef struct {
  uint8_t motor_id;
  uint32_t version;
} RobStrideVersionReply;

enum {
  ROBSTRIDE_PARAM_RUN_MODE = 0x7005,
  ROBSTRIDE_PARAM_IQ_REF = 0x7006,
  ROBSTRIDE_PARAM_SPEED_REF = 0x700A,
  ROBSTRIDE_PARAM_TORQUE_LIMIT = 0x700B,
  ROBSTRIDE_PARAM_POSITION_REF = 0x7016,
  ROBSTRIDE_PARAM_SPEED_LIMIT = 0x7017,
  ROBSTRIDE_PARAM_CURRENT_LIMIT = 0x7018,
  ROBSTRIDE_PARAM_POSITION = 0x7019,
  ROBSTRIDE_PARAM_IQ = 0x701A,
  ROBSTRIDE_PARAM_SPEED = 0x701B,
  ROBSTRIDE_PARAM_BUS_VOLTAGE = 0x701C,
  ROBSTRIDE_PARAM_ACCELERATION = 0x7022,
  ROBSTRIDE_PARAM_PROFILE_SPEED = 0x7024,
  ROBSTRIDE_PARAM_PROFILE_ACCELERATION = 0x7025,
  ROBSTRIDE_PARAM_REPORT_INTERVAL = 0x7026,
  ROBSTRIDE_PARAM_TIMEOUT = 0x7028
};

const RobStrideLimits *robstride_limits(RobStrideModel model);
uint32_t robstride_make_extended_id(RobStrideCommType type,
                                    uint16_t data_field,
                                    uint8_t target_id);
bool robstride_make_motion(RobStrideModel model, uint8_t motor_id,
                           const RobStrideMotionCommand *command,
                           CanFrame *out);
bool robstride_make_enable(uint8_t motor_id, uint8_t master_id, CanFrame *out);
bool robstride_make_stop(uint8_t motor_id, uint8_t master_id,
                         bool clear_fault, CanFrame *out);
bool robstride_make_set_zero(uint8_t motor_id, uint8_t master_id,
                             CanFrame *out);
bool robstride_make_discover(uint8_t master_id, CanFrame *out);
bool robstride_make_read_version(uint8_t motor_id, uint8_t master_id,
                                 CanFrame *out);
bool robstride_make_set_can_id(uint8_t motor_id, uint8_t new_motor_id,
                               uint8_t master_id, CanFrame *out);
bool robstride_make_save_parameters(uint8_t motor_id, uint8_t master_id,
                                    CanFrame *out);
bool robstride_make_set_baud(uint8_t motor_id, uint8_t master_id,
                             RobStrideBaud baud, CanFrame *out);
bool robstride_make_set_reporting(uint8_t motor_id, uint8_t master_id,
                                  bool enabled, CanFrame *out);
bool robstride_make_set_protocol(uint8_t motor_id, uint8_t master_id,
                                 RobStrideProtocol protocol, CanFrame *out);
bool robstride_make_read_parameter(uint8_t motor_id, uint8_t master_id,
                                   uint16_t index, CanFrame *out);
bool robstride_make_write_parameter_f32(uint8_t motor_id, uint8_t master_id,
                                        uint16_t index, float value,
                                        CanFrame *out);
bool robstride_make_write_parameter_u8(uint8_t motor_id, uint8_t master_id,
                                       uint16_t index, uint8_t value,
                                       CanFrame *out);
bool robstride_parse_feedback(RobStrideModel model, uint8_t master_id,
                              const CanFrame *frame,
                              RobStrideFeedback *out);
bool robstride_parse_parameter_reply(uint8_t master_id, const CanFrame *frame,
                                     RobStrideParameterReply *out);
bool robstride_parse_discovery_reply(const CanFrame *frame,
                                     RobStrideDiscoveryReply *out);
bool robstride_parse_fault_reply(uint8_t master_id, const CanFrame *frame,
                                 RobStrideFaultReply *out);
bool robstride_parse_version_reply(uint8_t master_id, const CanFrame *frame,
                                   RobStrideVersionReply *out);

#endif
