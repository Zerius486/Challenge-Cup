/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_hal_timebase_TIM.c 
  * @brief   HAL time base based on the hardware TIM.
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
 #include "stm32f4xx.h"
 #include "gpio.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef        htim5; 
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
TIM_HandleTypeDef  TIM_TimeBaseStructure;
TIM_OC_InitTypeDef TIM_OCInitStructure;
__IO uint16_t ChannelPulse = 500;

/**
  * @brief  配置TIM4复用输出PWM时用到的I/O
  * @param  PD12 PD13 PD14 PD15 
  * @retval tim4ch1
  */
static void TIM4_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
 
  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		
}

//PWM11  PC8   TIM3_CH3    控制材料挤出机
//PWM12  PC9   TIM3_CH4
static void TIM3_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
}
static void TIM3_PWMOUTPUT_Config(void)
{
 //TIM_BreakDeadTimeConfigTypeDef TIM_BDTRInitStructure;
  
	// 开启TIMx_CLK,x[1,8] 
	__HAL_RCC_TIM3_CLK_ENABLE();
	
	/* 定义定时器的句柄即确定定时器寄存器的基地址*/
	TIM_TimeBaseStructure.Instance = TIM3;
	/* 累计 TIM_Period个后产生一个更新或者中断*/		
	//当定时器从0计数到999，即为1000次，为一个定时周期
	TIM_TimeBaseStructure.Init.Period = 3000-1;
	
	// TIM3通用控制定时器时钟源TIMxCLK = HCLK/2=84MHz 
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=1000KHZ
	TIM_TimeBaseStructure.Init.Prescaler = 84-1;	
	// 采样时钟分频
	TIM_TimeBaseStructure.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	// 计数方式
	TIM_TimeBaseStructure.Init.CounterMode=TIM_COUNTERMODE_UP;
	// 重复计数器
	TIM_TimeBaseStructure.Init.RepetitionCounter=0;	
	// 初始化定时器TIMx, x[1,8]
	HAL_TIM_PWM_Init(&TIM_TimeBaseStructure);
	/*PWM模式配置*/

	/*PWM模式配置*/
	//配置为PWM模式1
	TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
	TIM_OCInitStructure.Pulse = 3000;
//	TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;	
	TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_LOW;
   // TIM_OCInitStructure.

	//初始化通道1输出PWM 
	HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure,&TIM_OCInitStructure,TIM_CHANNEL_1);
	//初始化通道2输出PWM 
		TIM_OCInitStructure.Pulse = 0;
	HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure,&TIM_OCInitStructure,TIM_CHANNEL_2);
	//初始化通道3输出PWM 
		TIM_OCInitStructure.Pulse = 1500;
	HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure,&TIM_OCInitStructure,TIM_CHANNEL_3);
	//初始化通道4输出PWM 
		TIM_OCInitStructure.Pulse = 3000;
	HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure,&TIM_OCInitStructure,TIM_CHANNEL_4);

	/* 定时器通道1输出PWM */
	HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_1);	
		/* 定时器通道2输出PWM */
	HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_2);	
	/* 定时器通道3输出PWM */
	HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_3);		
	/* 定时器通道4输出PWM */
	HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_4);			
}



//PWM5    PD12  TIM4_CH1  //冲洗水枪     900-2400。
//PWM6    PD13  TIM4_CH2  //水下磨刷900-2000pwm波
//PWM7    PD14  TIM4_CH3 // 水下钻头500-1500-2500
//PWM8    PD15  TIM4_CH4 //水下切割900-2400
static void TIM4_PWMOUTPUT_Config(void)
{
// TIM_BreakDeadTimeConfigTypeDef TIM_BDTRInitStructure;
  
	// 开启TIMx_CLK,x[1,8] 
	__HAL_RCC_TIM4_CLK_ENABLE();
	
	/* 定义定时器的句柄即确定定时器寄存器的基地址*/
	TIM_TimeBaseStructure.Instance = TIM4;
	/* 累计 TIM_Period个后产生一个更新或者中断*/		
	//当定时器从0计数到999，即为1000次，为一个定时周期
	TIM_TimeBaseStructure.Init.Period = 3000-1;
	
	// TIM8通用控制定时器时钟源TIMxCLK = HCLK/2=84MHz 
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=1000KHZ
	TIM_TimeBaseStructure.Init.Prescaler = 84-1;	
	// 采样时钟分频
	TIM_TimeBaseStructure.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	// 计数方式
	TIM_TimeBaseStructure.Init.CounterMode=TIM_COUNTERMODE_UP;
	// 重复计数器
	TIM_TimeBaseStructure.Init.RepetitionCounter=0;	
	// 初始化定时器TIMx, x[1,8]
	HAL_TIM_PWM_Init(&TIM_TimeBaseStructure);
	/*PWM模式配置*/

///* PWM1 Mode configuration: Channel1 */
//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
//  TIM_OCInitStructure.TIM_Pulse = 1500-1;
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  	  //当定时器计数值小于CCR1_Val时为高电平
//  
	/*PWM模式配置*/
	//配置为PWM模式1
	TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
	TIM_OCInitStructure.Pulse = 900;
//	TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
	
	TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_LOW;
   // TIM_OCInitStructure.

	//初始化通道1输出PWM 
	HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure,&TIM_OCInitStructure,TIM_CHANNEL_1);
	//初始化通道2输出PWM 
		TIM_OCInitStructure.Pulse = 1500;
	HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure,&TIM_OCInitStructure,TIM_CHANNEL_2);
	//初始化通道3输出PWM 
		TIM_OCInitStructure.Pulse = 1500;
	HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure,&TIM_OCInitStructure,TIM_CHANNEL_3);
	//初始化通道4输出PWM 
		TIM_OCInitStructure.Pulse = 900;
	HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure,&TIM_OCInitStructure,TIM_CHANNEL_4);

	/* 定时器通道1输出PWM */
	HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_1);	
		/* 定时器通道2输出PWM */
	HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_2);	
	/* 定时器通道3输出PWM */
	HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_3);		
	/* 定时器通道4输出PWM */
	HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_4);		
	
}
void TIM4_Configuration(void)
{
  TIM3_GPIO_Config(); 
  TIM3_PWMOUTPUT_Config();		
	
  TIM4_GPIO_Config(); 
  TIM4_PWMOUTPUT_Config();	
	
}







/**
  * @brief  This function configures the TIM5 as a time base source. 
  *         The time source is configured  to have 1ms time base with a dedicated 
  *         Tick interrupt priority. 
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig(). 
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock = 0;
  uint32_t              uwPrescalerValue = 0;
  uint32_t              pFLatency;
  
  /*Configure the TIM5 IRQ priority */
  HAL_NVIC_SetPriority(TIM5_IRQn, TickPriority ,0); 
  
  /* Enable the TIM5 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM5_IRQn); 
  
  /* Enable TIM5 clock */
  __HAL_RCC_TIM5_CLK_ENABLE();
  
  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
  
  /* Compute TIM5 clock */
  uwTimclock = 2*HAL_RCC_GetPCLK1Freq();
   
  /* Compute the prescaler value to have TIM5 counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000) - 1);
  
  /* Initialize TIM5 */
  htim5.Instance = TIM5;
  
  /* Initialize TIMx peripheral as follow:
  + Period = [(TIM5CLK/1000) - 1]. to have a (1/1000) s time base.
  + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
  + ClockDivision = 0
  + Counter direction = Up
  */
  htim5.Init.Period = (1000000 / 1000) - 1;
  htim5.Init.Prescaler = uwPrescalerValue;
  htim5.Init.ClockDivision = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&htim5) == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    return HAL_TIM_Base_Start_IT(&htim5);
  }
  
  /* Return function status */
  return HAL_ERROR;
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM5 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Disable TIM5 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_UPDATE);                                                  
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TIM5 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
  /* Enable TIM5 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
