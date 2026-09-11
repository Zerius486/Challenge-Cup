#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#define hInsideCan	hcan1	
unsigned char csBoadId = 0x02;
unsigned char pcBoadId = 0x04;

#include "jlCAN.h"
#include "JLDataType.h"
#include "InsideLed.h"

unsigned char CCtoCSSend(unsigned char _fuc);
unsigned char CCtoPCSend(unsigned char _fuc);

void InsideCanTask(unsigned char *argument)
{
	
	osDelay(500);
	jlCANBaudSet(&hInsideCan,250000);
//	jlCANFilterInit(&hInsideCan,csBoadId,0);
//	jlCANFilterInit(&hInsideCan,pcBoadId,1);
	HAL_CAN_Start(&hInsideCan);													//开始CAN
	HAL_CAN_ActivateNotification(&hInsideCan, CAN_IT_RX_FIFO0_MSG_PENDING);		//使能FIFO0接收中断
	for(;;)
	{
		CCtoPCSend(1);
		osDelay(2);
		CCtoPCSend(2);
		osDelay(2);
		CCtoPCSend(3);
		osDelay(2);
		CCtoPCSend(4);
		osDelay(2);
		CCtoPCSend(5);
		osDelay(2);
		CCtoPCSend(6);
		osDelay(2);
		CCtoPCSend(7);
		osDelay(2);
		CCtoPCSend(8);
		osDelay(2);
		CCtoPCSend(9);
		osDelay(17);
		CCtoCSSend(1);
		osDelay(17);
//		{
//			static unsigned char testCount = 0;
//			if(((testCount++)&0x0F) == 0)
//			{
//				DbgLedFlag = !DbgLedFlag;
//			}
//		}
	}
}

void PCtoCC_Decode(unsigned char _fuc,unsigned char *_dat);
unsigned char uctRxMsgData[8] = {0,};
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef xtRxMsgHeader;
	if(hcan->Instance == CAN1)
	{
		if(HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&xtRxMsgHeader,uctRxMsgData) == HAL_OK)
		{
			if((((xtRxMsgHeader.ExtId)>>4) & 0x3F) == pcBoadId)
			{
				PCtoCC_Decode(xtRxMsgHeader.ExtId & 0x0F ,uctRxMsgData);
			}
		}
	}
}

void PCtoCC_Decode(unsigned char _fuc,unsigned char *_dat)
{
	
	switch(_fuc)
	{
		case 1:			//ID1~ID8 表示的是8个推进器速度
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
			ExDatBuf[PIndxThterSped((_fuc-1))].fdata = *(float *)_dat;
		break;
		case 9:			//ID9表示330V电压
			ExDatBuf[PIndxP330VOLT].fdata = *(float *)_dat;
		break;
		case 10:		//ID10表示电池电压和330转化的48V电压
			ExDatBuf[PIndxCELLVOLT].fdata = *(float *)_dat;
			ExDatBuf[PIndxPI48VOLT].fdata = *(float *)(_dat+4);
		break;
		case 11:		//ID11表示电源舱温度和状态字
			ExDatBuf[PIndxPCabTmp].fdata = *(float *)_dat;
			ExDatBuf[PIndxPCabLeka].bdata = (*(_dat+4)&0x01)?'1':'0';
		break;
	}
}

unsigned char CCtoCSSend(unsigned char _fuc)
{
	CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBuf[8] = {0,};
	unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;

	xtTxHeader.ExtId = ((NodeMasterSlaveSendMask | csBoadId) << 4) | _fuc;
	xtTxHeader.IDE = CAN_ID_EXT;
	xtTxHeader.RTR = CAN_RTR_DATA;
	if(_fuc == 1)
	{
		xtTxHeader.DLC = 1;
		uctTxBuf[0] = 	((ExDatBuf[PIndxEthPwEn(0)].bdata-'0') << 0) |
						((ExDatBuf[PIndxEthPwEn(1)].bdata-'0') << 1) |
						((ExDatBuf[PIndxEthPwEn(2)].bdata-'0') << 2) |
						((ExDatBuf[PIndxEthPwEn(3)].bdata-'0') << 3) |
						((ExDatBuf[PIndxEthPwEn(4)].bdata-'0') << 4) |
						((ExDatBuf[PIndxEthPwEn(5)].bdata-'0') << 5) ;
	}
	else
	{
		return 1;
	}
	HAL_CAN_AddTxMessage(&hInsideCan, &xtTxHeader, uctTxBuf, &ultTxMailBox);
	return 0;
}

unsigned char CCtoPCSend(unsigned char _fuc)
{
	CAN_TxHeaderTypeDef xtTxHeader;
	unsigned char uctTxBuf[8] = {0,};
	unsigned int  ultTxMailBox = CAN_TX_MAILBOX0;

//	float _temp = 0.0f;
	xtTxHeader.ExtId = ((NodeMasterSlaveSendMask | pcBoadId) << 4) | _fuc;
	xtTxHeader.IDE = CAN_ID_EXT;
	xtTxHeader.RTR = CAN_RTR_DATA;
	
	switch(_fuc)
	{
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
			xtTxHeader.DLC = 4;
//			_temp = ExDatBuf[ThterCtrl((_fuc-1))].fdata;
			uctTxBuf[0] = *(((unsigned char *)&ExDatBuf[PIndxThterCtrl((_fuc-1))].fdata)+0);
			uctTxBuf[1] = *(((unsigned char *)&ExDatBuf[PIndxThterCtrl((_fuc-1))].fdata)+1);
			uctTxBuf[2] = *(((unsigned char *)&ExDatBuf[PIndxThterCtrl((_fuc-1))].fdata)+2);
			uctTxBuf[3] = *(((unsigned char *)&ExDatBuf[PIndxThterCtrl((_fuc-1))].fdata)+3);
		break;
		case 9:
			xtTxHeader.DLC = 1;
			uctTxBuf[0] |= 	ExDatBuf[PIndxPI48CVEN].bdata == '0'?0:0x01;
		break;
		default:
			return 1;
	}
	HAL_CAN_AddTxMessage(&hInsideCan, &xtTxHeader, uctTxBuf, &ultTxMailBox);
	return 0;	
}

