#ifndef __JL_CAN_H__
#define __JL_CAN_H__

#include "can.h"

#define NodeMasterSlave_Master		0x40
#define NodeMasterSlave_Slave		0
#define NodeMasterSlaveRecvMask			NodeMasterSlave_Master		//作为主机接收
#define NodeMasterSlaveSendMask			NodeMasterSlave_Slave		//向从机发送

unsigned char jlCANBaudSet(CAN_HandleTypeDef *hcan,unsigned int _baud);
void jlCANFilterInit(CAN_HandleTypeDef *hcan,unsigned char _id,unsigned char _fbank);
#endif /*__JL_CAN_H__*/
