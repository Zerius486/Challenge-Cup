#ifndef EROB_CANOPEN_H
#define EROB_CANOPEN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_frame.h"

#define EROB_NODE_ID_MIN 1U
#define EROB_NODE_ID_MAX 127U

typedef enum {
  EROB_MODE_PROFILE_POSITION = 1,
  EROB_MODE_PROFILE_VELOCITY = 3,
  EROB_MODE_PROFILE_TORQUE = 4,
  EROB_MODE_INTERPOLATED_POSITION = 7,
  EROB_MODE_CYCLIC_SYNC_POSITION = 8,
  EROB_MODE_CYCLIC_SYNC_VELOCITY = 9,
  EROB_MODE_CYCLIC_SYNC_TORQUE = 10
} ErobOperationMode;

typedef enum {
  EROB_NMT_START = 0x01,
  EROB_NMT_STOP = 0x02,
  EROB_NMT_PRE_OPERATIONAL = 0x80,
  EROB_NMT_RESET_NODE = 0x81,
  EROB_NMT_RESET_COMMUNICATION = 0x82
} ErobNmtCommand;

typedef enum {
  EROB_DS402_UNKNOWN = 0,
  EROB_DS402_NOT_READY,
  EROB_DS402_SWITCH_ON_DISABLED,
  EROB_DS402_READY_TO_SWITCH_ON,
  EROB_DS402_SWITCHED_ON,
  EROB_DS402_OPERATION_ENABLED,
  EROB_DS402_QUICK_STOP_ACTIVE,
  EROB_DS402_FAULT_REACTION_ACTIVE,
  EROB_DS402_FAULT
} ErobDs402State;

typedef enum {
  EROB_SDO_INVALID = 0,
  EROB_SDO_DOWNLOAD_OK,
  EROB_SDO_UPLOAD_OK,
  EROB_SDO_ABORT
} ErobSdoResultKind;

typedef struct {
  ErobSdoResultKind kind;
  uint8_t node_id;
  uint16_t index;
  uint8_t subindex;
  uint8_t value_size;
  uint32_t value;
  uint32_t abort_code;
} ErobSdoResult;

bool erob_make_nmt(ErobNmtCommand command, uint8_t node_id, CanFrame *out);
bool erob_make_sdo_read(uint8_t node_id, uint16_t index, uint8_t subindex,
                        CanFrame *out);
bool erob_make_sdo_write_u8(uint8_t node_id, uint16_t index, uint8_t subindex,
                            uint8_t value, CanFrame *out);
bool erob_make_sdo_write_u16(uint8_t node_id, uint16_t index, uint8_t subindex,
                             uint16_t value, CanFrame *out);
bool erob_make_sdo_write_u32(uint8_t node_id, uint16_t index, uint8_t subindex,
                             uint32_t value, CanFrame *out);
bool erob_parse_sdo_response(const CanFrame *frame, ErobSdoResult *out);

bool erob_make_mode(uint8_t node_id, ErobOperationMode mode, CanFrame *out);
bool erob_make_controlword(uint8_t node_id, uint16_t controlword,
                           CanFrame *out);
bool erob_make_target_position(uint8_t node_id, int32_t counts, CanFrame *out);
bool erob_make_target_velocity(uint8_t node_id, int32_t units, CanFrame *out);
bool erob_make_target_torque(uint8_t node_id, int16_t permille,
                             CanFrame *out);
bool erob_make_profile_velocity(uint8_t node_id, uint32_t units,
                                CanFrame *out);
bool erob_make_profile_acceleration(uint8_t node_id, uint32_t units,
                                    CanFrame *out);
bool erob_make_profile_deceleration(uint8_t node_id, uint32_t units,
                                    CanFrame *out);
bool erob_make_statusword_read(uint8_t node_id, CanFrame *out);
ErobDs402State erob_ds402_state(uint16_t statusword);
uint16_t erob_ds402_next_controlword(ErobDs402State state);

#endif
