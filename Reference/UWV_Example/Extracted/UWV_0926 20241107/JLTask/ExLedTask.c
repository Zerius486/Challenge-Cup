#include "ExLedTask.h"
#include "AD5328.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "JLDataType.h"




void ExLedTask(unsigned char *argument)
{
	osDelay(500);
	
	AD5328Init();

	osDelay(500);
	for(;;)
	{
		//通道1
		if(ExDatBuf[PIndxExLedPwEn(1)].bdata == '1')
		{
			ExLedPwEnableCh(1);
			AD5328Refresh(ExLedDimCh(1),(unsigned short)(ExDatBuf[PIndxExLedDimm(1)].fdata*65536.0f/5.01f));
		}
		else
		{
			ExLedPwDisableCh(1);
			AD5328Refresh(ExLedDimCh(1),0);
		}
		osDelay(20);
		//通道2
		if(ExDatBuf[PIndxExLedPwEn(2)].bdata == '1')
		{
			ExLedPwEnableCh(2);
			AD5328Refresh(ExLedDimCh(2),(unsigned short)(ExDatBuf[PIndxExLedDimm(2)].fdata*65536.0f/5.01f));
		}
		else
		{
			ExLedPwDisableCh(2);
			AD5328Refresh(ExLedDimCh(2),0);
		}
		osDelay(20);
		//通道3
		if(ExDatBuf[PIndxExLedPwEn(3)].bdata == '1')
		{
			ExLedPwEnableCh(3);
			AD5328Refresh(ExLedDimCh(3),(unsigned short)(ExDatBuf[PIndxExLedDimm(3)].fdata*65536.0f/5.01f));
		}
		else
		{
			ExLedPwDisableCh(3);
			AD5328Refresh(ExLedDimCh(3),0);
		}
		osDelay(20);
		//通道4
		if(ExDatBuf[PIndxExLedPwEn(4)].bdata == '1')
		{
			ExLedPwEnableCh(4);
			AD5328Refresh(ExLedDimCh(4),(unsigned short)(ExDatBuf[PIndxExLedDimm(4)].fdata*65536.0f/5.01f));
		}
		else
		{
			ExLedPwDisableCh(4);
			AD5328Refresh(ExLedDimCh(4),0);
		}
		osDelay(20);
		//通道5
		if(ExDatBuf[PIndxExLedPwEn(5)].bdata == '1')
		{
			ExLedPwEnableCh(5);
			AD5328Refresh(ExLedDimCh(5),(unsigned short)(ExDatBuf[PIndxExLedDimm(5)].fdata*65536.0f/5.01f));
		}
		else
		{
			ExLedPwDisableCh(5);
			AD5328Refresh(ExLedDimCh(5),0);
		}
		osDelay(20);
	}
}


