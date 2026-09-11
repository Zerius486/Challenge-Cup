/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "JlGpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId_t xRunLedTaskHandle;
osThreadId_t xEthRecvTaskHandle;
osThreadId_t xEthSendTaskHandle;
osThreadId_t xEthSetTaskHandle;
osThreadId_t xJointTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void RunLedTask(void *argument);
void EthRecvTask(void *argument);
void EthSendTask(void *argument);
void EthSetTask(void *argument);
void JointTask(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */
osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of xRunLedTask */
  const osThreadAttr_t xRunLedTask_attributes = {
    .name = "xRunLedTask",
    .priority = (osPriority_t) osPriorityLow,
    .stack_size = 2048
  };
  xRunLedTaskHandle = osThreadNew(RunLedTask, NULL, &xRunLedTask_attributes);

  /* definition and creation of xEthRecvTask */
  const osThreadAttr_t xEthRecvTask_attributes = {
    .name = "xEthRecvTask",
    .priority = (osPriority_t) osPriorityHigh7,
    .stack_size = 1024
  };
  xEthRecvTaskHandle = osThreadNew(EthRecvTask, NULL, &xEthRecvTask_attributes);

  /* definition and creation of xEthSendTask */
  const osThreadAttr_t xEthSendTask_attributes = {
    .name = "xEthSendTask",
    .priority = (osPriority_t) osPriorityHigh6,
    .stack_size = 1024
  };
  xEthSendTaskHandle = osThreadNew(EthSendTask, NULL, &xEthSendTask_attributes);

  /* definition and creation of xEthSetTask */
  const osThreadAttr_t xEthSetTask_attributes = {
    .name = "xEthSetTask",
    .priority = (osPriority_t) osPriorityHigh5,
    .stack_size = 512
  };
  xEthSetTaskHandle = osThreadNew(EthSetTask, NULL, &xEthSetTask_attributes);

  /* definition and creation of xJointTask */
  const osThreadAttr_t xJointTask_attributes = {
    .name = "xJointTask",
    .priority = (osPriority_t) osPriorityRealtime,
    .stack_size = 6200
  };
  xJointTaskHandle = osThreadNew(JointTask, NULL, &xJointTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_RunLedTask */
/**
  * @brief  Function implementing the xRunLedTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_RunLedTask */
__weak void RunLedTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN RunLedTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RunLedTask */
}

/* USER CODE BEGIN Header_EthRecvTask */
/**
* @brief Function implementing the xEthRecvTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EthRecvTask */
__weak void EthRecvTask(void *argument)
{
  /* USER CODE BEGIN EthRecvTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END EthRecvTask */
}

/* USER CODE BEGIN Header_EthSendTask */
/**
* @brief Function implementing the xEthSendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EthSendTask */
__weak void EthSendTask(void *argument)
{
  /* USER CODE BEGIN EthSendTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END EthSendTask */
}

/* USER CODE BEGIN Header_EthSetTask */
/**
* @brief Function implementing the xEthSetTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EthSetTask */
__weak void EthSetTask(void *argument)
{
  /* USER CODE BEGIN EthSetTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END EthSetTask */
}

/* USER CODE BEGIN Header_JointTask */
/**
* @brief Function implementing the xJointTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_JointTask */
__weak void JointTask(void *argument)
{
  /* USER CODE BEGIN JointTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END JointTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
