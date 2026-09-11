/*

*/

#include "jlCAN.h"
#include "CanMsg.h"

#include "string.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

unsigned char ucMsgRptPrd = 0;				//自动上报周期，单位为10ms

typedef struct
{
	unsigned int baud;
	unsigned int prescaler;
	unsigned int sjw; 
	unsigned int ts1;
	unsigned int ts2;
}CanBaudInitTypeDef;

//CAN通讯波特率列表（APB1=42M）
#define CanBaudTableMax		19
const CanBaudInitTypeDef CanBaudTable[CanBaudTableMax] = {	{1000000,3		,CAN_SJW_1TQ,CAN_BS1_7TQ,CAN_BS2_6TQ},		//1M
															{750000	,4		,CAN_SJW_1TQ,CAN_BS1_7TQ,CAN_BS2_6TQ},		//750K
															{500000	,6		,CAN_SJW_1TQ,CAN_BS1_7TQ,CAN_BS2_6TQ},		//500K
															{375000	,8		,CAN_SJW_1TQ,CAN_BS1_7TQ,CAN_BS2_6TQ},		//375K
															{300000	,10		,CAN_SJW_1TQ,CAN_BS1_7TQ,CAN_BS2_6TQ},		//300K
															{250000	,12		,CAN_SJW_1TQ,CAN_BS1_7TQ,CAN_BS2_6TQ},		//250K
															{200000	,15		,CAN_SJW_1TQ,CAN_BS1_7TQ,CAN_BS2_6TQ},		//200K
															{150000	,20		,CAN_SJW_1TQ,CAN_BS1_7TQ,CAN_BS2_6TQ},		//150K
															{125000	,24		,CAN_SJW_1TQ,CAN_BS1_7TQ,CAN_BS2_6TQ},		//125K
															{100000	,30		,CAN_SJW_1TQ,CAN_BS1_7TQ,CAN_BS2_6TQ},		//100K
															{75000	,40		,CAN_SJW_1TQ,CAN_BS1_7TQ,CAN_BS2_6TQ},		//75K
															{50000	,60		,CAN_SJW_1TQ,CAN_BS1_7TQ,CAN_BS2_6TQ},		//50K
															{25000	,120	,CAN_SJW_1TQ,CAN_BS1_7TQ,CAN_BS2_6TQ},		//25K
															{12500	,240	,CAN_SJW_1TQ,CAN_BS1_7TQ,CAN_BS2_6TQ},		//12.5K
															{10000	,300	,CAN_SJW_1TQ,CAN_BS1_7TQ,CAN_BS2_6TQ},		//10K
															{7500	,400	,CAN_SJW_1TQ,CAN_BS1_7TQ,CAN_BS2_6TQ},		//7.5K
															{5000	,600	,CAN_SJW_1TQ,CAN_BS1_7TQ,CAN_BS2_6TQ},		//5K
															{2500	,1200	,CAN_SJW_1TQ,CAN_BS1_7TQ,CAN_BS2_6TQ},		//2.5K
															{1000	,3000	,CAN_SJW_1TQ,CAN_BS1_7TQ,CAN_BS2_6TQ}};		//1K

/*
return 0,设置成功；1,不存在的波特率；2,设置失败
											*/
unsigned char jlCANBaudSet(CAN_HandleTypeDef *hcan,unsigned int _baud)
{
	unsigned char i = 0;
	
	while(_baud != CanBaudTable[i].baud)
	{
		if(i++ >= CanBaudTableMax)
		{
			return 1;
		}
	}
	
	hcan->Init.Prescaler = CanBaudTable[i].prescaler;
	hcan->Init.SyncJumpWidth = CanBaudTable[i].sjw;
	hcan->Init.TimeSeg1 = CanBaudTable[i].ts1;
	hcan->Init.TimeSeg2 = CanBaudTable[i].ts2;
	
	if (HAL_CAN_Init(hcan) != HAL_OK)
	{
		Error_Handler();
		return 2;
	}
	return 0;
}
/*
CAN滤波器初始化函数，适配鲸灵标准协议
20210406：更改适配主机和从机
*/
void jlCANFilterInit(CAN_HandleTypeDef *hcan,unsigned char _id,unsigned char _fbank)
{
	CAN_FilterTypeDef sFilterConfig;
	unsigned int ultIDList = 0;
	unsigned int ultIDMask = 0;
	unsigned char uctFrameType = CAN_ID_STD;
	
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	if(hcan->Instance == CAN1)
	{
		sFilterConfig.FilterFIFOAssignment = 0;
	}
	else
	{
		sFilterConfig.FilterFIFOAssignment = 1;
	}
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 0;
	
	ultIDMask = 0x1FFFFFF0;
	uctFrameType = CAN_ID_EXT;
	
	//配置过滤器0	接收目的地址为本设备的指令
	ultIDList = (NodeMasterSlaveRecvMask | (_id & 0x3F)) << 4;		//20210406,更改后适用于主机从机初始化
	if(_fbank < 14)
	{
		sFilterConfig.FilterBank = _fbank;
	}
	else
	{
		sFilterConfig.FilterBank = 0;
	}
	
	if(uctFrameType == CAN_ID_STD)
	{
		sFilterConfig.FilterIdHigh = (ultIDList<<5)&0xFFE0;			//((IDList<<5)&0xFFE0)|((IDList>>24)&0x1F)
		sFilterConfig.FilterIdLow = CAN_ID_STD | CAN_RTR_DATA;		//((IDList>>10)&0xFFFC) | CAN_ID_EXT | CAN_RTR_DATA | 0
		sFilterConfig.FilterMaskIdHigh = (ultIDMask<<5)&0xFFE0;		//[ID10-ID0]|[ID28-ID24]
		sFilterConfig.FilterMaskIdLow = 0x04 | 0x02;				//[ID23-ID12]|IDE|RTR|0
	}
	else if(uctFrameType == CAN_ID_EXT)
	{
		sFilterConfig.FilterIdHigh = (ultIDList>>13) & 0xFFFF;									
		sFilterConfig.FilterIdLow = ((ultIDList<<3)&0xFFF8) | CAN_ID_EXT | CAN_RTR_DATA;		
		sFilterConfig.FilterMaskIdHigh = (ultIDMask>>13) & 0xFFFF;		
		sFilterConfig.FilterMaskIdLow = ((ultIDMask<<3)&0xFFF8) | 0x04 | 0x02;	
	}
	HAL_CAN_ConfigFilter(hcan, &sFilterConfig);	
}
