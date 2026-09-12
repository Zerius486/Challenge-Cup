#ifndef __CAN2_H
#define __CAN2_H	 
#include "common.h"	 

//////////////////////////////////////////////////////////////////////////////////	 

extern CAN_HandleTypeDef	CAN2_Handler;     //CAN1句柄
	
					    
										 							 				    
u8 CAN2_Mode_Init(u32 tsjw,u32 tbs2,u32 tbs1,u16 brp,u32 mode);//CAN初始化
void CAN2_Config(void); 
u8 CAN2_Send_Msg(u8* msg,u8 len);						//发送数据

u8 CAN2_Receive_Msg(u8 *buf);							//接收数据
#endif

















