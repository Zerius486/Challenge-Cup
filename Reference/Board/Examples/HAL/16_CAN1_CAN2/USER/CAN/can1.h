#ifndef __CAN1_H
#define __CAN1_H	 
#include "common.h"	 

//////////////////////////////////////////////////////////////////////////////////	 


							    
										 							 				    
u8 CAN1_Mode_Init(u32 tsjw,u32 tbs2,u32 tbs1,u16 brp,u32 mode);//CAN初始化
void CAN1_Config(void); 
u8 CAN1_Send_Msg(u8* msg,u8 len);						//发送数据

u8 CAN1_Receive_Msg(u8 *buf);							//接收数据
#endif

















