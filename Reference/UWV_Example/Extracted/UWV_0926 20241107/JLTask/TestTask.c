#include "InsideLed.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

void EthSendPack(void);

void TestTask(unsigned char *argument)
{
	osDelay(500);

	for(;;)
	{
		osDelay(1000);
	}
}
