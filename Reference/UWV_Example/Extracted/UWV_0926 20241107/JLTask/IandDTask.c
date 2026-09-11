#include "gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "JLDataType.h"
#include "INS660.h"


void IandDTask(unsigned char *argument)
{
	USART6->CR1 |= USART_CR1_RXNEIE;
	osDelay(500);
	GpioReset(IaDPw);
	osDelay(500);
	
	for(;;)
	{
		if(ExDatBuf[PIndxIanDPwEn].bdata == '1')
		{
			GpioSet(IaDPw);
//			Ins660StartRecv();
			Ins660Decode();
			ExDatBuf[PIndxIanDAngleVelo(0)].fdata = ins660XAngleVelo;
			ExDatBuf[PIndxIanDAngleVelo(1)].fdata = ins660YAngleVelo;
			ExDatBuf[PIndxIanDAngleVelo(2)].fdata = ins660ZAngleVelo;
			ExDatBuf[PIndxIanDAcc(0)].fdata = ins660XAcce;
			ExDatBuf[PIndxIanDAcc(1)].fdata = ins660YAcce;
			ExDatBuf[PIndxIanDAcc(2)].fdata = ins660ZAcce;
			ExDatBuf[PIndxIanDSpeed(0)].fdata = ins660NorthSpeed;
			ExDatBuf[PIndxIanDSpeed(1)].fdata = ins660SkySpeed;
			ExDatBuf[PIndxIanDSpeed(2)].fdata = ins660EastSpeed;
			ExDatBuf[PIndxIanDAltitude].fdata = ins660Altitude;
			ExDatBuf[PIndxIanDLongitude].idata = ins660Longitude;
			ExDatBuf[PIndxIanDLatitude].idata = ins660Latitude;
			ExDatBuf[PIndxIanDRoll].fdata = ins660Roll;
			ExDatBuf[PIndxIanDYaw].fdata = ins660Yaw;
			ExDatBuf[PIndxIanDPitch].fdata = ins660Pitch;
			ExDatBuf[PIndxIanDHeave].idata = ins660Heave;
		}
		else 
		{
			GpioReset(IaDPw);
		}
		osDelay(1);
	}
}

