
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "KellerBus.h"
#include "JLDataType.h"

#include "gpio.h"
#define DepthPw_High()			{DepPw_GPIO_Port->ODR |= DepPw_Pin;}
#define DepthPw_Low()			{DepPw_GPIO_Port->ODR &= ~DepPw_Pin;}

void DepthTask(unsigned char *argument)
{
	osDelay(2000);
	KellerBusSendCmd(48);
	osDelay(2000);
	KellerBusSendCmd(48);
	osDelay(500);
	for(;;)
	{
		if(ExDatBuf[PIndxDepthPwEn].bdata == '1')
		{
			DepthPw_High();
			if(!KellerBusSendCmd(73))
			{
				ExDatBuf[PIndxDepthData].fdata = KellerBusDat;
			}
			else
			{
				osDelay(1000);
				KellerBusSendCmd(48);
			}
		}
		else if(ExDatBuf[PIndxDepthPwEn].bdata == '0')
		{
			DepthPw_Low();
		}
		
		osDelay(1000);
	}
}



