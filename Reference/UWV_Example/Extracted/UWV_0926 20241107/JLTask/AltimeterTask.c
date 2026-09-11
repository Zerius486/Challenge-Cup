//#include "AltimeterTask.h"
#include <string.h>

#include "ISA500.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "gpio.h"
#include "usart.h"

#define AltPw_High()			{AltPw_GPIO_Port->ODR |= AltPw_Pin;}
#define AltPw_Low()				{AltPw_GPIO_Port->ODR &= ~AltPw_Pin;}

#include "JLDataType.h"

#define altOptStep_Stoped		0
#define altOptStep_Start		1
#define altOptStep_Run			2
#define altOptStep_Stop			3

void AltimeterTask(unsigned char *argument)
{
	unsigned char altOptStep = altOptStep_Stoped;
	osDelay(500);
	
	AltPw_Low();
	osDelay(100);

	for(;;)
	{
		switch(altOptStep)
		{
			case altOptStep_Stoped:
				if(ExDatBuf[PIndxAltPwEn].bdata == '1')
				{
					AltPw_High();
					altOptStep = altOptStep_Start;
					osDelay(100);
				}
				else
				{
					osDelay(100);
				}
			break;
			case altOptStep_Start:
				ISA500RecvStart();
				altOptStep = altOptStep_Run;
				osDelay(100);
			break;
			case altOptStep_Run:
				if(ISA500BusErrFlag == 1)	//接收到错误数据，证明电源已经开启，只要重启接收就可以
				{
					ISA500BusErrFlag = 0;
					ISA500RecvStart();
				}
				
				if((ISA500OverTimeFlag++ > 200) || (ExDatBuf[PIndxAltPwEn].bdata != '1'))		//2s内未接收到数据，或者要求停止高度计
				{
					altOptStep = altOptStep_Stop;
					ISA500RecvStop();
				}
				
				if((ISA500OverTimeFlag/10)%2)
				{
					ISA500RecvStop();
					osDelay(100);
					ISA500RecvStart();
				}
				ExDatBuf[PIndxAltData].fdata = altDat;
				ExDatBuf[PIndxAltTmpData].fdata = altTmpDat;
				
				osDelay(100);
			break;
			case altOptStep_Stop:
				AltPw_Low();
				altOptStep = altOptStep_Stoped;
				osDelay(100);
			break;
			default:
				altOptStep = altOptStep_Stop;
			break;
		}

	}
}
