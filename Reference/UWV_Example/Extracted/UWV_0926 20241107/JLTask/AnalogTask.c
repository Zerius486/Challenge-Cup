#include "Analog.h"
#include "PT1000.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "JLDataType.h"




void AnalogTask(unsigned char *argument)
{
	osDelay(500);
	AnalogCovStart();
	osDelay(500);
	
	for(;;)
	{
		
		osDelay(1000);
	}
}

