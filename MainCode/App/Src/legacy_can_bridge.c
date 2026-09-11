#include "legacy_can_bridge.h"

#include <string.h>

#include "byte_codec.h"

/** @brief 解码兼容 UDP CAN 桥接的 15 字节数据。 */
bool legacy_can_bridge_decode(const uint8_t *data, size_t length,
                              CanFrame *out) {
  uint32_t id;
  if (data == NULL || out == NULL || length != LEGACY_CAN_BRIDGE_FRAME_SIZE ||
      (data[0] != 0U && data[0] != 4U) ||
      (data[1] != 0U && data[1] != 2U) || data[6] > 8U) {
    return false;
  }
  id = codec_read_u32_be(&data[2]);
  if ((data[0] == 0U && id > 0x7FFU) ||
      (data[0] == 4U && id > 0x1FFFFFFFU)) {
    return false;
  }
  memset(out, 0, sizeof(*out));
  out->id = id;
  out->is_extended = data[0] == 4U;
  out->is_remote = data[1] == 2U;
  out->dlc = data[6];
  memcpy(out->data, &data[7], 8U);
  return true;
}

/** @brief 将统一 CAN 帧编码为兼容桥接格式。 */
size_t legacy_can_bridge_encode(const CanFrame *frame,
                                uint8_t out[LEGACY_CAN_BRIDGE_FRAME_SIZE]) {
  if (frame == NULL || out == NULL || frame->dlc > 8U ||
      (!frame->is_extended && frame->id > 0x7FFU) ||
      (frame->is_extended && frame->id > 0x1FFFFFFFU)) {
    return 0U;
  }
  memset(out, 0, LEGACY_CAN_BRIDGE_FRAME_SIZE);
  out[0] = frame->is_extended ? 4U : 0U;
  out[1] = frame->is_remote ? 2U : 0U;
  codec_write_u32_be(&out[2], frame->id);
  out[6] = frame->dlc;
  memcpy(&out[7], frame->data, 8U);
  return LEGACY_CAN_BRIDGE_FRAME_SIZE;
}

/** @brief 判断兼容桥接帧是否为允许的急停帧。 */
bool legacy_can_bridge_is_safe_stop(const CanFrame *frame) {
  uint8_t type;
  uint8_t target_id;
  uint16_t index;
  uint16_t controlword;

  if (frame == NULL || frame->is_remote || frame->dlc != 8U) {
    return false;
  }
  if (frame->is_extended) {
    type = (uint8_t)((frame->id >> 24) & 0x1FU);
    target_id = (uint8_t)frame->id;
    return type == 0x04U && target_id >= 1U && target_id <= 127U &&
           frame->data[0] <= 1U && frame->data[1] == 0U &&
           frame->data[2] == 0U && frame->data[3] == 0U &&
           frame->data[4] == 0U && frame->data[5] == 0U &&
           frame->data[6] == 0U && frame->data[7] == 0U;
  }
  if (frame->id < 0x601U || frame->id > 0x67FU ||
      frame->data[0] != 0x2BU || frame->data[3] != 0U ||
      frame->data[6] != 0U || frame->data[7] != 0U) {
    return false;
  }
  index = codec_read_u16_le(&frame->data[1]);
  controlword = codec_read_u16_le(&frame->data[4]);
  return index == 0x6040U && (controlword == 0U || controlword == 2U);
}
