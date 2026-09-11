#include "erob_canopen.h"

#include <string.h>

#include "byte_codec.h"

#define EROB_SDO_REQUEST_BASE 0x600U
#define EROB_SDO_RESPONSE_BASE 0x580U

static bool valid_node(uint8_t node_id) {
  return node_id >= EROB_NODE_ID_MIN && node_id <= EROB_NODE_ID_MAX;
}

static bool make_sdo(uint8_t node_id, uint8_t command, uint16_t index,
                     uint8_t subindex, uint32_t value, CanFrame *out) {
  if (!valid_node(node_id) || out == NULL) {
    return false;
  }

  memset(out, 0, sizeof(*out));
  out->id = EROB_SDO_REQUEST_BASE + node_id;
  out->dlc = 8U;
  out->is_extended = false;
  out->data[0] = command;
  codec_write_u16_le(&out->data[1], index);
  out->data[3] = subindex;
  codec_write_u32_le(&out->data[4], value);
  return true;
}

/** @brief 构造 CANopen NMT 网络管理帧。 */
bool erob_make_nmt(ErobNmtCommand command, uint8_t node_id, CanFrame *out) {
  if (out == NULL || node_id > EROB_NODE_ID_MAX) {
    return false;
  }
  switch (command) {
    case EROB_NMT_START:
    case EROB_NMT_STOP:
    case EROB_NMT_PRE_OPERATIONAL:
    case EROB_NMT_RESET_NODE:
    case EROB_NMT_RESET_COMMUNICATION:
      break;
    default:
      return false;
  }

  memset(out, 0, sizeof(*out));
  out->id = 0U;
  out->dlc = 2U;
  out->is_extended = false;
  out->data[0] = (uint8_t)command;
  out->data[1] = node_id;
  return true;
}

/** @brief 构造 CANopen SDO 上传请求。 */
bool erob_make_sdo_read(uint8_t node_id, uint16_t index, uint8_t subindex,
                        CanFrame *out) {
  return make_sdo(node_id, 0x40U, index, subindex, 0U, out);
}

/** @brief 构造 1 字节 SDO 下载请求。 */
bool erob_make_sdo_write_u8(uint8_t node_id, uint16_t index, uint8_t subindex,
                            uint8_t value, CanFrame *out) {
  return make_sdo(node_id, 0x2FU, index, subindex, value, out);
}

/** @brief 构造 2 字节 SDO 下载请求。 */
bool erob_make_sdo_write_u16(uint8_t node_id, uint16_t index, uint8_t subindex,
                             uint16_t value, CanFrame *out) {
  return make_sdo(node_id, 0x2BU, index, subindex, value, out);
}

/** @brief 构造 4 字节 SDO 下载请求。 */
bool erob_make_sdo_write_u32(uint8_t node_id, uint16_t index, uint8_t subindex,
                             uint32_t value, CanFrame *out) {
  return make_sdo(node_id, 0x23U, index, subindex, value, out);
}

/** @brief 解析 eRob SDO 上传、下载确认或 abort 响应。 */
bool erob_parse_sdo_response(const CanFrame *frame, ErobSdoResult *out) {
  uint8_t command;

  if (frame == NULL || out == NULL || frame->is_extended || frame->is_remote ||
      frame->dlc != 8U || frame->id <= EROB_SDO_RESPONSE_BASE ||
      frame->id > EROB_SDO_RESPONSE_BASE + EROB_NODE_ID_MAX) {
    return false;
  }

  memset(out, 0, sizeof(*out));
  out->node_id = (uint8_t)(frame->id - EROB_SDO_RESPONSE_BASE);
  out->index = codec_read_u16_le(&frame->data[1]);
  out->subindex = frame->data[3];
  command = frame->data[0];

  if (command == 0x60U) {
    out->kind = EROB_SDO_DOWNLOAD_OK;
    return true;
  }
  if (command == 0x80U) {
    out->kind = EROB_SDO_ABORT;
    out->abort_code = codec_read_u32_le(&frame->data[4]);
    return true;
  }

  if ((command & 0xE0U) != 0x40U || (command & 0x02U) == 0U) {
    return false;
  }

  out->kind = EROB_SDO_UPLOAD_OK;
  out->value_size = ((command & 0x01U) != 0U)
                        ? (uint8_t)(4U - ((command >> 2) & 0x03U))
                        : 4U;
  if (out->value_size == 0U || out->value_size > 4U) {
    return false;
  }
  out->value = codec_read_u32_le(&frame->data[4]);
  return true;
}

/** @brief 构造 eRob 工作模式写入帧。 */
bool erob_make_mode(uint8_t node_id, ErobOperationMode mode, CanFrame *out) {
  switch (mode) {
    case EROB_MODE_PROFILE_POSITION:
    case EROB_MODE_PROFILE_VELOCITY:
    case EROB_MODE_PROFILE_TORQUE:
    case EROB_MODE_INTERPOLATED_POSITION:
    case EROB_MODE_CYCLIC_SYNC_POSITION:
    case EROB_MODE_CYCLIC_SYNC_VELOCITY:
    case EROB_MODE_CYCLIC_SYNC_TORQUE:
      break;
    default:
      return false;
  }
  return erob_make_sdo_write_u8(node_id, 0x6060U, 0U, (uint8_t)mode, out);
}

/** @brief 构造 CiA 402 控制字写入帧。 */
bool erob_make_controlword(uint8_t node_id, uint16_t controlword,
                           CanFrame *out) {
  return erob_make_sdo_write_u16(node_id, 0x6040U, 0U, controlword, out);
}

/** @brief 构造 eRob 绝对编码器位置目标帧。 */
bool erob_make_target_position(uint8_t node_id, int32_t counts, CanFrame *out) {
  return erob_make_sdo_write_u32(node_id, 0x607AU, 0U, (uint32_t)counts, out);
}

/** @brief 构造 eRob 速度目标帧。 */
bool erob_make_target_velocity(uint8_t node_id, int32_t units, CanFrame *out) {
  return erob_make_sdo_write_u32(node_id, 0x60FFU, 0U, (uint32_t)units, out);
}

/** @brief 构造 eRob 目标转矩（额定电流千分比）帧。 */
bool erob_make_target_torque(uint8_t node_id, int16_t permille,
                             CanFrame *out) {
  return erob_make_sdo_write_u16(node_id, 0x6071U, 0U,
                                 (uint16_t)permille, out);
}

/** @brief 构造 eRob 轮廓速度帧。 */
bool erob_make_profile_velocity(uint8_t node_id, uint32_t units,
                                CanFrame *out) {
  return erob_make_sdo_write_u32(node_id, 0x6081U, 0U, units, out);
}

/** @brief 构造 eRob 轮廓加速度帧。 */
bool erob_make_profile_acceleration(uint8_t node_id, uint32_t units,
                                    CanFrame *out) {
  return erob_make_sdo_write_u32(node_id, 0x6083U, 0U, units, out);
}

/** @brief 构造 eRob 轮廓减速度帧。 */
bool erob_make_profile_deceleration(uint8_t node_id, uint32_t units,
                                    CanFrame *out) {
  return erob_make_sdo_write_u32(node_id, 0x6084U, 0U, units, out);
}

/** @brief 构造读取 eRob 状态字的 SDO 请求。 */
bool erob_make_statusword_read(uint8_t node_id, CanFrame *out) {
  return erob_make_sdo_read(node_id, 0x6041U, 0U, out);
}

/** @brief 将 CiA 402 状态字转换为可读状态枚举。 */
ErobDs402State erob_ds402_state(uint16_t statusword) {
  if ((statusword & 0x004FU) == 0x0000U) {
    return EROB_DS402_NOT_READY;
  }
  if ((statusword & 0x004FU) == 0x0040U) {
    return EROB_DS402_SWITCH_ON_DISABLED;
  }
  if ((statusword & 0x006FU) == 0x0021U) {
    return EROB_DS402_READY_TO_SWITCH_ON;
  }
  if ((statusword & 0x006FU) == 0x0023U) {
    return EROB_DS402_SWITCHED_ON;
  }
  if ((statusword & 0x006FU) == 0x0027U) {
    return EROB_DS402_OPERATION_ENABLED;
  }
  if ((statusword & 0x006FU) == 0x0007U) {
    return EROB_DS402_QUICK_STOP_ACTIVE;
  }
  if ((statusword & 0x004FU) == 0x000FU) {
    return EROB_DS402_FAULT_REACTION_ACTIVE;
  }
  if ((statusword & 0x004FU) == 0x0008U) {
    return EROB_DS402_FAULT;
  }
  return EROB_DS402_UNKNOWN;
}

/** @brief 根据当前 CiA 402 状态给出下一步控制字。 */
uint16_t erob_ds402_next_controlword(ErobDs402State state) {
  switch (state) {
    case EROB_DS402_SWITCH_ON_DISABLED:
      return 0x0006U;
    case EROB_DS402_READY_TO_SWITCH_ON:
      return 0x0007U;
    case EROB_DS402_SWITCHED_ON:
    case EROB_DS402_QUICK_STOP_ACTIVE:
      return 0x000FU;
    case EROB_DS402_FAULT:
      return 0x0080U;
    default:
      return 0U;
  }
}
