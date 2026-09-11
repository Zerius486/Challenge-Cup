#include "CanMsg.h"
#include "jlCAN.h"
#include "JointTask.h"
#include "task.h"
//CAN1 RX PD0
//CAN1 TX PD1

extern void EthSendPack(CAN_RxHeaderTypeDef _canRxMsgHeader,unsigned char *_RxBuf);
void CanFilterInit(CAN_HandleTypeDef *hcan);

/**
  * @brief  CAN总线初始化
  * @param  NULL
  * @retval NULL
**/
void CanInit(void)
{
	CanFilterInit(&hcan1);
	HAL_CAN_Start(&hcan1);
	/* CAN1 ---> FIFO1 */
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
	CanFilterInit(&hcan2);										//CAN总线滤波器配置
	HAL_CAN_Start(&hcan2);													//开始CAN
	/* CAN2 ---> FIFO0 */
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);		//使能FIFO0接收中断
}

/**
  * @brief  CAN总线滤波器初始化,设置为接收所有CAN消息
  * @param  CAN_HandleTypeDef *hcan
  * @retval NULL
**/
void CanFilterInit(CAN_HandleTypeDef *hcan)
{
	CAN_FilterTypeDef sFilterConfig;
	
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	if (hcan->Instance == CAN1)
	{
		sFilterConfig.FilterBank = 0;
		sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	}
	else
	{
		sFilterConfig.FilterBank = 14;
		sFilterConfig.FilterFIFOAssignment = CAN_FilterFIFO0;
	}
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	sFilterConfig.FilterIdHigh = 0;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0;
	sFilterConfig.FilterMaskIdLow = 0;
	HAL_CAN_ConfigFilter(hcan, &sFilterConfig);	
}

unsigned char uctRxMsgData[8] = {0,};
unsigned char djRxMsgData[8] = {0,};
//--------电机数据-----------------------------------
int32_t odoBuf[2] = {0, 0};						//相对里程
uint32_t odoAbsBuf[2] = {0, 0};				//绝对里程
int32_t rpmDataBuf[2] = {0, 0};				//电转速
float curDataBuf[2] = {0, 0};					//电流
//---------------------------------------------------------

/**
  * @brief  CAN1_Rx1Fifo1回调函数，接收电机和力传感器数据
  * @param  CAN_HandleTypeDef *hcan
  * @retval NULL
**/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef djRxMsgHeader;
	
	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO1, &djRxMsgHeader, djRxMsgData) == HAL_OK)
	{	
		uint8_t mos = djRxMsgHeader.ExtId >> 16;
		
		if (mos == 0x00)
		{
			mot_RxMessage_Receive(djRxMsgHeader, djRxMsgData);
		}
		else if (mos == 0x01)
		{
			fs_RxMsg_Receive(djRxMsgHeader, djRxMsgData);
		}
	}
}

/**
  * @brief  CAN2_Rx1Fifo0回调函数，接收机械臂数据
  * @param  CAN_HandleTypeDef *hcan
  * @retval NULL
**/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint32_t _temp = 0;	
			
	CAN_RxHeaderTypeDef xtRxMsgHeader;
	if(hcan->Instance == CAN2)
	{
		if(HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &xtRxMsgHeader, uctRxMsgData) == HAL_OK)
		{
			
			//开启TPDO之后的数据接收处理位置，电流
			if(xtRxMsgHeader.StdId == 0x18B)
			{		  
				xtRxMsgHeader.StdId = 0x0;		
				NowP[11] = (uctRxMsgData[0] | (uctRxMsgData[1]<<8) | (uctRxMsgData[2]<<16) | (uctRxMsgData[3]<<24));
				NowPosition[11] = NowP[11];	
				NowC[11] =(uctRxMsgData[4] | (uctRxMsgData[5]<<8) );	
			}
			else if(xtRxMsgHeader.StdId == 0x18C)
			{		
				xtRxMsgHeader.StdId = 0x0;			
				NowP[12] = (uctRxMsgData[0] | (uctRxMsgData[1]<<8) | (uctRxMsgData[2]<<16) | (uctRxMsgData[3]<<24));
				NowPosition[12] = NowP[12];		
				NowC[12]= (uctRxMsgData[4] | (uctRxMsgData[5]<<8) );	
			}		
			else if(xtRxMsgHeader.StdId == 0x18D)
			{			
				xtRxMsgHeader.StdId = 0x0;		
				NowP[13] =(uctRxMsgData[0] | (uctRxMsgData[1]<<8) | (uctRxMsgData[2]<<16) | (uctRxMsgData[3]<<24));	
				NowPosition[13] = NowP[13];	
				NowC[13] = (uctRxMsgData[4] | (uctRxMsgData[5]<<8));	
			}		
			else if(xtRxMsgHeader.StdId == 0x18A)
			{		
				xtRxMsgHeader.StdId = 0x0;		
				NowP[10] = (uctRxMsgData[0] | (uctRxMsgData[1]<<8) | (uctRxMsgData[2]<<16) | (uctRxMsgData[3]<<24));
				NowPosition[10] =	NowP[10]; 		
				NowC[10] = (uctRxMsgData[4] | (uctRxMsgData[5]<<8));	
			}
			else if( xtRxMsgHeader.StdId == 0x5cc  )//电压
			{		
				xtRxMsgHeader.StdId = 0x0;		
				if((uctRxMsgData[0] == 0x00) && (uctRxMsgData[4] == 0x3e))	//V
				{
					_temp = uctRxMsgData[3] | (uctRxMsgData[2]<<8) | (uctRxMsgData[1]<<16) | (uctRxMsgData[0]<<24);			
					NowV[0] = _temp;		
				}
			}		
			
			else
			{
			}
			
					
// else if(xtRxMsgHeader.ExtId == 0x00010220)
//    {
//		
//	//   _temp32 = _RxBuf[1] | (_RxBuf[0]<<8) | (_RxBuf[3]<<16) | (_RxBuf[2]<<24);		

//_temp32=_RxBuf[2];
//_temp32=(_temp32<<8)+_RxBuf[3];
//_temp32=(_temp32<<8)+_RxBuf[0];
//_temp32=(_temp32<<8)+_RxBuf[1];					
//	   NowT[0] = _temp32;		//f_x_t
//			
//_temp32=_RxBuf[6];
//_temp32=(_temp32<<8)+_RxBuf[7];
//_temp32=(_temp32<<8)+_RxBuf[4];
//_temp32=(_temp32<<8)+_RxBuf[5];					
////	   _temp32 = _RxBuf[5] | (_RxBuf[4]<<8) | (_RxBuf[7]<<16) | (_RxBuf[6]<<24);			
//	   NowT[1] = _temp32;		//f_Y_t
//		
//    }
		
		}

	}	
}


//-----CAN数据处理函数---------------------------------------------------------
void buffer_append_int32(uint8_t* buffer, int32_t number, int8_t* index)
{
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_float32(uint8_t* buffer, float number, int8_t* index)
{
	buffer_append_int32(buffer, (int32_t)number, index);
}

int16_t  buffer_get_int16(const uint8_t* buffer, int8_t* index)
{
    int16_t res = ((uint16_t)buffer[*index]) << 8 	| 
									((uint16_t)buffer[(*index) + 1]);
    *index += 2;
    return res;
}

int32_t buffer_get_int32(const uint8_t* buffer, int8_t* index)
{
    int32_t res = ((uint32_t)buffer[*index]) << 24			|
									((uint32_t)buffer[*index + 1] << 16) 	|
                	((uint32_t)buffer[*index + 2] << 8) 	|
									((uint32_t)buffer[*index + 3]);
    *index += 4;
    return res;
}

uint32_t buffer_get_uint32(const uint8_t* buffer, int8_t* index)
{
    uint32_t res = ((uint32_t)buffer[*index]) << 24			|
									((uint32_t)buffer[*index + 1] << 16) 	|
                	((uint32_t)buffer[*index + 2] << 8) 	|
									((uint32_t)buffer[*index + 3]);
	*index += 4;
	return res;
}
//--------------------------------------------------------------------------

//----------CAN数据发送函数--------------------------------------------------------------
void comm_can_transmit(uint32_t id, uint8_t* data, uint8_t len)
{
	CAN_TxHeaderTypeDef CAN_TxMsgData;
	uint32_t CAN_TxMailBox = CAN_TX_MAILBOX0;
	CAN_TxMsgData.ExtId = id;
	CAN_TxMsgData.DLC = len;
	CAN_TxMsgData.IDE = CAN_ID_EXT;
	CAN_TxMsgData.RTR = CAN_RTR_DATA ;
	HAL_CAN_AddTxMessage(&hcan1, &CAN_TxMsgData, data, &CAN_TxMailBox);
}

void comm_can_set_rpm(uint8_t controller_id, int32_t rpm) 
{
	int8_t ind = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, rpm, &ind);	
	//taskENTER_CRITICAL();
	comm_can_transmit(controller_id |((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, 4);	
	//taskEXIT_CRITICAL();
}

void comm_can_get_odo(uint8_t controller_id)
{
	uint8_t buffer[3] = {0x08, 0x00, 0x04};
	if (controller_id == 0x09)
	{
		buffer[0] = 0x09;
	}
	comm_can_transmit(controller_id | ((uint32_t)CAN_PACKET_ASK_DATA << 8), buffer, 3);
}
//-------------------------------------------------------------------------------

//获取力传感器数据
void fs_RxMsg_Receive(CAN_RxHeaderTypeDef fsRxMsgHeader, unsigned char fsRxMsgData[8])
{
	int32_t _temp32 = 0;
	if(fsRxMsgHeader.ExtId == 0x00010280)
	{
		fsRxMsgHeader.ExtId = 0x0;			
		
		_temp32 = fsRxMsgData[2];
		_temp32 = (_temp32<<8) + fsRxMsgData[3];
		_temp32 = (_temp32<<8) + fsRxMsgData[0];
		_temp32 = (_temp32<<8) + fsRxMsgData[1];				
		NowT[4] = _temp32;		//f_Y
		_temp32 = fsRxMsgData[6];
		_temp32 = (_temp32<<8) + fsRxMsgData[7];
		_temp32 = (_temp32<<8) + fsRxMsgData[4];
		_temp32 = (_temp32<<8) + fsRxMsgData[5];		
		NowT[5] = _temp32;		//f_Z		
	}			
	else if(fsRxMsgHeader.ExtId == 0x00010241)
	{
		fsRxMsgHeader.ExtId = 0x0;			
		_temp32 = fsRxMsgData[2];
		_temp32 = (_temp32<<8)+uctRxMsgData[3];
		_temp32 = (_temp32<<8)+uctRxMsgData[0];
		_temp32 = (_temp32<<8)+uctRxMsgData[1];				
		NowT[2] = _temp32;		//f_Z_t
		_temp32 = fsRxMsgData[6];
		_temp32 = (_temp32<<8) + fsRxMsgData[7];
		_temp32 = (_temp32<<8) + fsRxMsgData[4];
		_temp32 = (_temp32<<8) + fsRxMsgData[5];	
		NowT[3] = _temp32;		//f_x
	}
}

//获取电机数据
void mot_RxMessage_Receive(CAN_RxHeaderTypeDef djRxMsgHeader, unsigned char motRxMsgData[8])
{
	int8_t index;
	uint32_t cmd = djRxMsgHeader.ExtId >> 8;
	
	if (cmd == 0x05)
	{
		if (djRxMsgHeader.ExtId == CAN_ODO_ID1)
		{
			if (motRxMsgData[0] == 0x2A)
			{
				//0x2A		--->		byte5 --- byte8
				index = 4;
				odoBuf[0] = buffer_get_int32(motRxMsgData, &index);
			}
			else if (motRxMsgData[0] == 0x31)
			{
				//0x31		--->		byte2	---	byte5
				index = 1;
				odoAbsBuf[0] = buffer_get_uint32(motRxMsgData, &index);
			}
		}
			
		else if (djRxMsgHeader.ExtId == CAN_ODO_ID2)
		{
			if (motRxMsgData[0] == 0x2A)
			{
				index = 4;
				odoBuf[1] = buffer_get_int32(motRxMsgData, &index);
			}
			else if (motRxMsgData[0] == 0x31)
			{
				index = 1;
				odoAbsBuf[1] = buffer_get_uint32(motRxMsgData, &index);					}
		}
	}
			
	//VESC状态自动发送
	#if CAN_STATUS_ENABLE
	else if (cmd == 0x09)
	{
		if (djRxMsgHeader.ExtId == 0x00000908)
		{
			//index = 0;
			//rpmDataBuf[0] = buffer_get_int32(motRxMsgData, &index);
			index = 4;
			curDataBuf[0] = (float)buffer_get_int16(motRxMsgData, &index) / 10.0;
		}
				
		else if (djRxMsgHeader.ExtId == 0x0000000909)
		{
			//index = 0;
			//rpmDataBuf[1] = buffer_get_int32(motRxMsgData, &index);
			index = 4;
			curDataBuf[1] = (float)buffer_get_int16(motRxMsgData, &index) / 10.0;
		}
	}
	#endif
}