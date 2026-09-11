#ifndef __CAN_MSG_H__
#define __CAN_MSG_H__


#include "stdint.h"
#include "can.h"

extern int32_t odoBuf[2];												//相对里程
extern uint32_t odoAbsBuf[2];										//绝对里程
extern int32_t rpmDataBuf[2];										//电转速
extern float curDataBuf[2];											//电流
extern void CanInit(void);

struct kinematic_model{													
	int32_t odo_last[2];
	int32_t odo_now[2];
	int32_t odo_inc[2];
	float position[3];
	float ang_inc;
	float angle;
	float degree;
};

void buffer_append_int32(uint8_t* buffer, int32_t number, int8_t* index);
void buffer_append_float32(uint8_t* buffer, float number, int8_t* index);
int16_t  buffer_get_int16(const uint8_t* buffer, int8_t* index);
int32_t buffer_get_int32(const uint8_t* buffer, int8_t* index);
uint32_t buffer_get_uint32(const uint8_t* buffer, int8_t* index);
void comm_can_set_rpm(uint8_t vesc_id, int32_t rpm);
void comm_can_get_odo(uint8_t controller_id);
void mot_RxMessage_Receive(CAN_RxHeaderTypeDef djRxMsgHeader, unsigned char motRxMsgData[8]);
void fs_RxMsg_Receive(CAN_RxHeaderTypeDef fsRxMsgHeader, unsigned char fsRxMsgData[8]);
#define CAN_PACKET_SET_RPM		0x000003
#define CAN_PACKET_ASK_DATA   0x000008
#define CAN_PACKET_STATUS	 		0x000009
#define CAN_ODO_ID1						0x00000508
#define CAN_ODO_ID2						0x00000509
#define CAN_STATUS_ENABLE			01
#define CTRL_ID_1							0x08
#define CTRL_ID_2			    		0x09

#endif /*__CAN_MSG_H__*/
