#include "robstride.h"

#include <math.h>
#include <string.h>

#include "byte_codec.h"

static const RobStrideLimits k_limits[ROBSTRIDE_MODEL_COUNT] = {
    {33.0F, 14.0F, 16.0F, 500.0F, 5.0F},
    {44.0F, 17.0F, 23.0F, 500.0F, 5.0F},
    {44.0F, 17.0F, 16.0F, 500.0F, 5.0F},
    {20.0F, 60.0F, 43.0F, 5000.0F, 100.0F},
    {15.0F, 120.0F, 90.0F, 5000.0F, 100.0F},
    {50.0F, 5.5F, 11.0F, 500.0F, 5.0F},
    {50.0F, 36.0F, 57.0F, 5000.0F, 100.0F},
};

static uint16_t encode_linear(float value, float minimum, float maximum) {
  float scaled;
  if (value < minimum) {
    value = minimum;
  } else if (value > maximum) {
    value = maximum;
  }
  scaled = (value - minimum) * 65535.0F / (maximum - minimum);
  return (uint16_t)(scaled + 0.5F);
}

static float decode_linear(uint16_t raw, float minimum, float maximum) {
  return ((float)raw * (maximum - minimum) / 65535.0F) + minimum;
}

static bool make_simple(RobStrideCommType type, uint8_t motor_id,
                        uint8_t master_id, CanFrame *out) {
  if (out == NULL || motor_id == 0U || motor_id > 127U) {
    return false;
  }
  memset(out, 0, sizeof(*out));
  out->id = robstride_make_extended_id(type, master_id, motor_id);
  out->dlc = 8U;
  out->is_extended = true;
  return true;
}

/** @brief 返回指定 RobStride 型号的运动和电流限制。 */
const RobStrideLimits *robstride_limits(RobStrideModel model) {
  if ((unsigned)model >= ROBSTRIDE_MODEL_COUNT) {
    return NULL;
  }
  return &k_limits[model];
}

uint32_t robstride_make_extended_id(RobStrideCommType type,
                                    uint16_t data_field,
                                    uint8_t target_id) {
  return (((uint32_t)type & 0x1FU) << 24) |
         ((uint32_t)data_field << 8) | target_id;
}

/** @brief 构造 RobStride 扩展 CAN 运动控制帧。 */
bool robstride_make_motion(RobStrideModel model, uint8_t motor_id,
                           const RobStrideMotionCommand *command,
                           CanFrame *out) {
  const RobStrideLimits *limits = robstride_limits(model);
  uint16_t torque;

  if (limits == NULL || command == NULL || out == NULL || motor_id == 0U ||
      motor_id > 127U ||
      !isfinite(command->position_rad) || !isfinite(command->velocity_rad_s) ||
      !isfinite(command->torque_nm) || !isfinite(command->kp) ||
      !isfinite(command->kd) ||
      command->position_rad < -ROBSTRIDE_POSITION_LIMIT_RAD ||
      command->position_rad > ROBSTRIDE_POSITION_LIMIT_RAD ||
      command->velocity_rad_s < -limits->velocity_rad_s ||
      command->velocity_rad_s > limits->velocity_rad_s ||
      command->torque_nm < -limits->torque_nm ||
      command->torque_nm > limits->torque_nm || command->kp < 0.0F ||
      command->kp > limits->kp || command->kd < 0.0F ||
      command->kd > limits->kd) {
    return false;
  }

  memset(out, 0, sizeof(*out));
  torque = encode_linear(command->torque_nm, -limits->torque_nm,
                         limits->torque_nm);
  out->id = robstride_make_extended_id(ROBSTRIDE_COMM_MOTION, torque,
                                       motor_id);
  out->dlc = 8U;
  out->is_extended = true;
  codec_write_u16_be(&out->data[0],
                     encode_linear(command->position_rad,
                                   -ROBSTRIDE_POSITION_LIMIT_RAD,
                                   ROBSTRIDE_POSITION_LIMIT_RAD));
  codec_write_u16_be(&out->data[2],
                     encode_linear(command->velocity_rad_s,
                                   -limits->velocity_rad_s,
                                   limits->velocity_rad_s));
  codec_write_u16_be(&out->data[4],
                     encode_linear(command->kp, 0.0F, limits->kp));
  codec_write_u16_be(&out->data[6],
                     encode_linear(command->kd, 0.0F, limits->kd));
  return true;
}

/** @brief 构造 RobStride 使能帧。 */
bool robstride_make_enable(uint8_t motor_id, uint8_t master_id, CanFrame *out) {
  return make_simple(ROBSTRIDE_COMM_ENABLE, motor_id, master_id, out);
}

/** @brief 构造 RobStride 停止或清故障帧。 */
bool robstride_make_stop(uint8_t motor_id, uint8_t master_id,
                         bool clear_fault, CanFrame *out) {
  if (!make_simple(ROBSTRIDE_COMM_STOP, motor_id, master_id, out)) {
    return false;
  }
  out->data[0] = clear_fault ? 1U : 0U;
  return true;
}

/** @brief 构造 RobStride 设置零点帧。 */
bool robstride_make_set_zero(uint8_t motor_id, uint8_t master_id,
                             CanFrame *out) {
  if (!make_simple(ROBSTRIDE_COMM_SET_ZERO, motor_id, master_id, out)) {
    return false;
  }
  out->data[0] = 1U;
  return true;
}

/** @brief 构造 RobStride 广播发现帧。 */
bool robstride_make_discover(uint8_t master_id, CanFrame *out) {
  return make_simple(ROBSTRIDE_COMM_DISCOVER, 0x7FU, master_id, out);
}

/** @brief 构造 RobStride 版本读取帧。 */
bool robstride_make_read_version(uint8_t motor_id, uint8_t master_id,
                                 CanFrame *out) {
  if (!make_simple(ROBSTRIDE_COMM_STOP, motor_id, master_id, out)) {
    return false;
  }
  out->data[1] = 0xC4U;
  return true;
}

/** @brief 构造 RobStride 修改 CAN ID 帧。 */
bool robstride_make_set_can_id(uint8_t motor_id, uint8_t new_motor_id,
                               uint8_t master_id, CanFrame *out) {
  if (out == NULL || motor_id == 0U || motor_id > 127U ||
      new_motor_id == 0U || new_motor_id > 127U) {
    return false;
  }
  memset(out, 0, sizeof(*out));
  out->id = robstride_make_extended_id(
      ROBSTRIDE_COMM_SET_CAN_ID,
      (uint16_t)(((uint16_t)new_motor_id << 8) | master_id), motor_id);
  out->dlc = 8U;
  out->is_extended = true;
  return true;
}

/** @brief 构造 RobStride 保存参数帧。 */
bool robstride_make_save_parameters(uint8_t motor_id, uint8_t master_id,
                                    CanFrame *out) {
  static const uint8_t unlock[8] = {1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};
  if (!make_simple(ROBSTRIDE_COMM_SAVE, motor_id, master_id, out)) {
    return false;
  }
  memcpy(out->data, unlock, sizeof(unlock));
  return true;
}

static bool make_admin_command(RobStrideCommType type, uint8_t motor_id,
                               uint8_t master_id, uint8_t command,
                               CanFrame *out) {
  static const uint8_t unlock[6] = {1U, 2U, 3U, 4U, 5U, 6U};
  if (!make_simple(type, motor_id, master_id, out)) {
    return false;
  }
  memcpy(out->data, unlock, sizeof(unlock));
  out->data[6] = command;
  return true;
}

/** @brief 构造 RobStride 设置波特率帧。 */
bool robstride_make_set_baud(uint8_t motor_id, uint8_t master_id,
                             RobStrideBaud baud, CanFrame *out) {
  if (baud < ROBSTRIDE_BAUD_1M || baud > ROBSTRIDE_BAUD_125K) {
    return false;
  }
  return make_admin_command(ROBSTRIDE_COMM_SET_BAUD, motor_id, master_id,
                            (uint8_t)baud, out);
}

/** @brief 构造 RobStride 主动上报开关帧。 */
bool robstride_make_set_reporting(uint8_t motor_id, uint8_t master_id,
                                  bool enabled, CanFrame *out) {
  return make_admin_command(ROBSTRIDE_COMM_REPORT, motor_id, master_id,
                            enabled ? 1U : 0U, out);
}

/** @brief 构造 RobStride 协议模式切换帧。 */
bool robstride_make_set_protocol(uint8_t motor_id, uint8_t master_id,
                                 RobStrideProtocol protocol, CanFrame *out) {
  if ((unsigned)protocol > ROBSTRIDE_PROTOCOL_MIT) {
    return false;
  }
  return make_admin_command(ROBSTRIDE_COMM_PROTOCOL, motor_id, master_id,
                            (uint8_t)protocol, out);
}

/** @brief 构造 RobStride 参数读取帧。 */
bool robstride_make_read_parameter(uint8_t motor_id, uint8_t master_id,
                                   uint16_t index, CanFrame *out) {
  if (!make_simple(ROBSTRIDE_COMM_READ_PARAM, motor_id, master_id, out)) {
    return false;
  }
  codec_write_u16_le(&out->data[0], index);
  return true;
}

/** @brief 构造 RobStride 浮点参数写入帧。 */
bool robstride_make_write_parameter_f32(uint8_t motor_id, uint8_t master_id,
                                        uint16_t index, float value,
                                        CanFrame *out) {
  if (!isfinite(value) ||
      !make_simple(ROBSTRIDE_COMM_WRITE_PARAM, motor_id, master_id, out)) {
    return false;
  }
  codec_write_u16_le(&out->data[0], index);
  codec_write_f32_le(&out->data[4], value);
  return true;
}

/** @brief 构造 RobStride 8 位参数写入帧。 */
bool robstride_make_write_parameter_u8(uint8_t motor_id, uint8_t master_id,
                                       uint16_t index, uint8_t value,
                                       CanFrame *out) {
  if (!make_simple(ROBSTRIDE_COMM_WRITE_PARAM, motor_id, master_id, out)) {
    return false;
  }
  codec_write_u16_le(&out->data[0], index);
  out->data[4] = value;
  return true;
}

/** @brief 解析 RobStride 周期反馈中的位置、速度、力矩和温度。 */
bool robstride_parse_feedback(RobStrideModel model, uint8_t master_id,
                              const CanFrame *frame,
                              RobStrideFeedback *out) {
  const RobStrideLimits *limits = robstride_limits(model);
  uint8_t type;
  uint16_t data_field;

  if (limits == NULL || frame == NULL || out == NULL || !frame->is_extended ||
      frame->is_remote || frame->dlc != 8U ||
      (uint8_t)frame->id != master_id) {
    return false;
  }
  type = (uint8_t)((frame->id >> 24) & 0x1FU);
  if (type != ROBSTRIDE_COMM_FEEDBACK && type != ROBSTRIDE_COMM_REPORT) {
    return false;
  }

  memset(out, 0, sizeof(*out));
  data_field = (uint16_t)(frame->id >> 8);
  out->motor_id = (uint8_t)data_field;
  if (out->motor_id == 0U || out->motor_id > 127U) {
    return false;
  }
  out->fault_bits = (uint8_t)((data_field >> 8) & 0x3FU);
  out->mode_state = (uint8_t)((data_field >> 14) & 0x03U);
  out->position_rad = decode_linear(codec_read_u16_be(&frame->data[0]),
                                    -ROBSTRIDE_POSITION_LIMIT_RAD,
                                    ROBSTRIDE_POSITION_LIMIT_RAD);
  out->velocity_rad_s = decode_linear(codec_read_u16_be(&frame->data[2]),
                                      -limits->velocity_rad_s,
                                      limits->velocity_rad_s);
  out->torque_nm = decode_linear(codec_read_u16_be(&frame->data[4]),
                                 -limits->torque_nm, limits->torque_nm);
  out->temperature_c = (float)codec_read_u16_be(&frame->data[6]) * 0.1F;
  return true;
}

bool robstride_parse_parameter_reply(uint8_t master_id, const CanFrame *frame,
                                     RobStrideParameterReply *out) {
  uint8_t type;
  uint16_t data_field;

  if (frame == NULL || out == NULL || !frame->is_extended || frame->is_remote ||
      frame->dlc != 8U || (uint8_t)frame->id != master_id) {
    return false;
  }
  type = (uint8_t)((frame->id >> 24) & 0x1FU);
  if (type != ROBSTRIDE_COMM_READ_PARAM) {
    return false;
  }

  memset(out, 0, sizeof(*out));
  data_field = (uint16_t)(frame->id >> 8);
  out->type = (RobStrideCommType)type;
  out->motor_id = (uint8_t)data_field;
  if (out->motor_id == 0U || out->motor_id > 127U) {
    return false;
  }
  out->success = (uint8_t)(data_field >> 8) == 0U;
  out->index = codec_read_u16_le(&frame->data[0]);
  out->raw_value = codec_read_u32_le(&frame->data[4]);
  out->float_value = codec_read_f32_le(&frame->data[4]);
  return true;
}

bool robstride_parse_discovery_reply(const CanFrame *frame,
                                     RobStrideDiscoveryReply *out) {
  if (frame == NULL || out == NULL || !frame->is_extended || frame->is_remote ||
      frame->dlc != 8U ||
      ((frame->id >> 24) & 0x1FU) != ROBSTRIDE_COMM_DISCOVER ||
      (uint8_t)frame->id != 0xFEU) {
    return false;
  }
  out->motor_id = (uint8_t)(frame->id >> 8);
  if (out->motor_id == 0U || out->motor_id > 127U) {
    return false;
  }
  memcpy(out->uid, frame->data, sizeof(out->uid));
  return true;
}

bool robstride_parse_fault_reply(uint8_t master_id, const CanFrame *frame,
                                 RobStrideFaultReply *out) {
  if (frame == NULL || out == NULL || !frame->is_extended || frame->is_remote ||
      frame->dlc != 8U ||
      ((frame->id >> 24) & 0x1FU) != ROBSTRIDE_COMM_FAULT ||
      (uint8_t)frame->id != master_id) {
    return false;
  }
  out->motor_id = (uint8_t)(frame->id >> 8);
  if (out->motor_id == 0U || out->motor_id > 127U) {
    return false;
  }
  out->faults = codec_read_u32_le(&frame->data[0]);
  out->warnings = codec_read_u32_le(&frame->data[4]);
  return true;
}

bool robstride_parse_version_reply(uint8_t master_id, const CanFrame *frame,
                                   RobStrideVersionReply *out) {
  if (frame == NULL || out == NULL || !frame->is_extended || frame->is_remote ||
      frame->dlc != 8U ||
      ((frame->id >> 24) & 0x1FU) != ROBSTRIDE_COMM_FEEDBACK ||
      (uint8_t)frame->id != master_id || frame->data[0] != 0x00U ||
      frame->data[1] != 0xC4U || frame->data[2] != 0x56U) {
    return false;
  }
  out->motor_id = (uint8_t)(frame->id >> 8);
  if (out->motor_id == 0U || out->motor_id > 127U) {
    return false;
  }
  out->version = codec_read_u32_be(&frame->data[3]);
  return true;
}
