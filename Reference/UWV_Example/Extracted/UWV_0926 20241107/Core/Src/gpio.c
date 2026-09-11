/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */
#include "JlGpio.h"
/* USER CODE END 0 */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_adc.h"
/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/


__IO uint16_t ADC_ConvertedValue[RHEOSTAT_NOFCHANEL]={0};
DMA_HandleTypeDef DMA_Init_Handle;
ADC_HandleTypeDef ADC_Handle;
ADC_ChannelConfTypeDef ADC_Config;


//adc1 pf8  adc3_in4
//adc2 pf9  adc3_in5
//adc3 pf10  adc3_in6
//adc4 pf11  adc3_in7
//adc5 pc0  adc123_in10
//adc6 pc2  adc123_in12
//adc7 pc3  adc123_in13
//adc8 pa4  adc12_in4
//adc9 pa5  adc12_in5
//adc10 pa6  adc12_in6
//adc11 pb0  adc12_in8
//adc12 pb1  adc12_in9

//舵机1 pa0  adc123_in0
//舵机2 pa3  adc123_in3

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PD0   ------> CAN1_RX
     PD1   ------> CAN1_TX
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, RedLed_Pin|BlueLed_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOG, GreenLed_Pin, GPIO_PIN_SET);
	
	
  /*Configure GPIO pins : PFPin PFPin PFPin */
//  GPIO_InitStruct.Pin = RedLed_Pin|GreenLed_Pin|BlueLed_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
 

  GPIO_InitStruct.Pin = GreenLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);


  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}





static void Rheostat_ADC_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /*=====================adc3通道4, pf6板上adc1======================*/
    // 使能 GPIO 时钟
    __GPIOF_CLK_ENABLE();    
    // 配置 IO
    GPIO_InitStructure.Pin = GPIO_PIN_6;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;	    
    GPIO_InitStructure.Pull = GPIO_NOPULL ; //不上拉不下拉
    HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);	
    /*=====================adc3通道5, pf7板上adc2======================*/
    // 使能 GPIO 时钟
    __GPIOF_CLK_ENABLE();    
    // 配置 IO
    GPIO_InitStructure.Pin = GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;	    
    GPIO_InitStructure.Pull = GPIO_NOPULL ; //不上拉不下拉
    HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);	
    /*=====================adc3通道6, pf8板上adc3======================*/
    // 使能 GPIO 时钟
    __GPIOF_CLK_ENABLE();    
    // 配置 IO
    GPIO_InitStructure.Pin = GPIO_PIN_8;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;	    
    GPIO_InitStructure.Pull = GPIO_NOPULL ; //不上拉不下拉
    HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);	
    /*=====================adc3通道7, pf9板上adc4======================*/
    // 使能 GPIO 时钟
    __GPIOF_CLK_ENABLE();    
    // 配置 IO
    GPIO_InitStructure.Pin = GPIO_PIN_9;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;	    
    GPIO_InitStructure.Pull = GPIO_NOPULL ; //不上拉不下拉
    HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);	
  

/*=====================adc123通道10, pc0板上adc5======================*/
    // 使能 GPIO 时钟
    __GPIOC_CLK_ENABLE();    
    // 配置 IO
    GPIO_InitStructure.Pin = GPIO_PIN_0;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;	    
    GPIO_InitStructure.Pull = GPIO_NOPULL ; //不上拉不下拉
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
    /*=====================adc123通道12, pc2板上adc6======================*/
    // 使能 GPIO 时钟
    __GPIOC_CLK_ENABLE();    
    // 配置 IO
    GPIO_InitStructure.Pin = GPIO_PIN_2;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;	    
    GPIO_InitStructure.Pull = GPIO_NOPULL ; //不上拉不下拉
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
    /*=====================adc123通道13, pc3板上adc7======================*/
    // 使能 GPIO 时钟
    __GPIOC_CLK_ENABLE();    
    // 配置 IO
    GPIO_InitStructure.Pin = GPIO_PIN_3;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;	    
    GPIO_InitStructure.Pull = GPIO_NOPULL ; //不上拉不下拉
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  

/*=====================adc12通道4, pA4板上adc8======================*/
    // 使能 GPIO 时钟
    __GPIOA_CLK_ENABLE();    
    // 配置 IO
    GPIO_InitStructure.Pin = GPIO_PIN_4;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;	    
    GPIO_InitStructure.Pull = GPIO_NOPULL ; //不上拉不下拉
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*=====================adc12通道5, pA5板上adc9======================*/
    // 使能 GPIO 时钟
    __GPIOA_CLK_ENABLE();    
    // 配置 IO
    GPIO_InitStructure.Pin = GPIO_PIN_5;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;	    
    GPIO_InitStructure.Pull = GPIO_NOPULL ; //不上拉不下拉
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);	
    /*=====================adc12通道6, pA6板上adc10======================*/
	    // 使能 GPIO 时钟
    __GPIOA_CLK_ENABLE();    
    // 配置 IO
    GPIO_InitStructure.Pin = GPIO_PIN_6;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;	    
    GPIO_InitStructure.Pull = GPIO_NOPULL ; //不上拉不下拉
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);	
  
  /*=====================adc12通道8, pb0板上adc11======================*/
    // 使能 GPIO 时钟
    __GPIOB_CLK_ENABLE();    
    // 配置 IO
    GPIO_InitStructure.Pin = GPIO_PIN_0;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;	    
    GPIO_InitStructure.Pull = GPIO_NOPULL ; //不上拉不下拉
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /*=====================adc12通道9, pb1板上adc12======================*/
    // 使能 GPIO 时钟
    __GPIOB_CLK_ENABLE();    
    // 配置 IO
    GPIO_InitStructure.Pin = GPIO_PIN_1;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;	    
    GPIO_InitStructure.Pull = GPIO_NOPULL ; //不上拉不下拉
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /*=====================adc123通道0  pa0板上舵机1======================*/
    // 使能 GPIO 时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
	
    // 配置 IO
    GPIO_InitStructure.Pin = GPIO_PIN_0;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;	    
    GPIO_InitStructure.Pull = GPIO_NOPULL ; //不上拉不下拉
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);  

    /*=====================adc123通道3  pa3板上舵机2======================*/
    // 使能 GPIO 时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
	
    // 配置 IO
    GPIO_InitStructure.Pin = GPIO_PIN_3;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;	    
    GPIO_InitStructure.Pull = GPIO_NOPULL ; //不上拉不下拉
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 



}

static void Rheostat_ADC_Mode_Config(void)
{

    // ------------------DMA Init 结构体参数 初始化--------------------------
    // ADC1使用DMA2，数据流0，通道0，这个是手册固定死的
    // 开启DMA时钟
    __HAL_RCC_DMA2_CLK_ENABLE();
    // 数据传输通道
    DMA_Init_Handle.Instance = DMA2_Stream0;
    // 数据传输方向为外设到存储器	
    DMA_Init_Handle.Init.Direction = DMA_PERIPH_TO_MEMORY;	
    // 外设寄存器只有一个，地址不用递增
    DMA_Init_Handle.Init.PeriphInc = DMA_PINC_DISABLE;
    // 存储器地址固定
    DMA_Init_Handle.Init.MemInc = DMA_MINC_ENABLE; 
    // // 外设数据大小为半字，即两个字节 
    DMA_Init_Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; 
    //	存储器数据大小也为半字，跟外设数据大小相同
    DMA_Init_Handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;	
    // 循环传输模式
    DMA_Init_Handle.Init.Mode = DMA_CIRCULAR;
    // DMA 传输通道优先级为高，当使用一个DMA通道时，优先级设置不影响
    DMA_Init_Handle.Init.Priority = DMA_PRIORITY_HIGH;
    // 禁止DMA FIFO	，使用直连模式
    DMA_Init_Handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;  
    // FIFO 大小，FIFO模式禁止时，这个不用配置	
    DMA_Init_Handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    DMA_Init_Handle.Init.MemBurst = DMA_MBURST_SINGLE;
    DMA_Init_Handle.Init.PeriphBurst = DMA_PBURST_SINGLE;  
    // 选择 DMA 通道，通道存在于流中
    DMA_Init_Handle.Init.Channel = DMA_CHANNEL_0; 
    //初始化DMA流，流相当于一个大的管道，管道里面有很多通道
    HAL_DMA_Init(&DMA_Init_Handle); 

    HAL_DMA_Start (&DMA_Init_Handle,RHEOSTAT_ADC_DR_ADDR,(uint32_t)&ADC_ConvertedValue,RHEOSTAT_NOFCHANEL);

    // 开启ADC时钟
    __HAL_RCC_ADC1_CLK_ENABLE();
    // -------------------ADC Init 结构体 参数 初始化------------------------
    // ADC1
    ADC_Handle.Instance = ADC1;
    // 时钟为fpclk 4分频	
    ADC_Handle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
    // ADC 分辨率
    ADC_Handle.Init.Resolution = ADC_RESOLUTION_12B;
    // 扫描模式，多通道采集才需要	
    ADC_Handle.Init.ScanConvMode = ENABLE; 
    // 连续转换	
    ADC_Handle.Init.ContinuousConvMode = ENABLE;
    // 非连续转换	
    ADC_Handle.Init.DiscontinuousConvMode = DISABLE;
    // 非连续转换个数
    ADC_Handle.Init.NbrOfDiscConversion   = 0;
    //禁止外部边沿触发    
    ADC_Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    //使用软件触发，外部触发不用配置，注释掉即可
    //ADC_Handle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
    //数据右对齐	
    ADC_Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    //转换通道个数
    ADC_Handle.Init.NbrOfConversion = RHEOSTAT_NOFCHANEL;
    //使能连续转换请求
    ADC_Handle.Init.DMAContinuousRequests = ENABLE;
    //转换完成标志
    ADC_Handle.Init.EOCSelection          = DISABLE;    
    // 初始化ADC	                          
    HAL_ADC_Init(&ADC_Handle);
    //---------------------------------------------------------------------------
    // 配置 ADC1 通道0转换顺序为1，第一个转换，采样时间为3个时钟周期，舵机1
    ADC_Config.Channel      = ADC_CHANNEL_0;
    ADC_Config.Rank         = 1;    
    ADC_Config.SamplingTime = ADC_SAMPLETIME_3CYCLES;// 采样时间间隔	
    ADC_Config.Offset       = 0;
    HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config);
    
//    // 配置 ADC1 通道3转换顺序为2，第二个转换，采样时间为3个时钟周期，舵机2
//    ADC_Config.Channel      = ADC_CHANNEL_3;
//    ADC_Config.Rank         = 2;
//    ADC_Config.SamplingTime = ADC_SAMPLETIME_3CYCLES; // 采样时间间隔	
//    ADC_Config.Offset       = 0;
//    HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config);
  

//    // 配置 ADC1 通道9转换顺序为3，第三个转换，采样时间为3个时钟周期，板上adc12
//    ADC_Config.Channel      = ADC_CHANNEL_9;
//    ADC_Config.Rank         = 3;    	
//    ADC_Config.SamplingTime = ADC_SAMPLETIME_3CYCLES;// 采样时间间隔，
//    ADC_Config.Offset       = 0;
//    HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config);
//   
//   // 配置 ADC1 通道8转换顺序为4，采样时间为3个时钟周期，板上adc11
//    ADC_Config.Channel      = ADC_CHANNEL_8;
//    ADC_Config.Rank         = 4;    	
//    ADC_Config.SamplingTime = ADC_SAMPLETIME_3CYCLES;// 采样时间间隔
//    ADC_Config.Offset       = 0;
//    HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config);

//   // 配置 ADC1 通道6转换顺序为5，采样时间为3个时钟周期，板上adc10
//    ADC_Config.Channel      = ADC_CHANNEL_6;
//    ADC_Config.Rank         = 5;    	
//    ADC_Config.SamplingTime = ADC_SAMPLETIME_3CYCLES;// 采样时间间隔
//    ADC_Config.Offset       = 0;
//    HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config);

//   // 配置 ADC1 通道5转换顺序为6，采样时间为3个时钟周期，板上adc9
//    ADC_Config.Channel      = ADC_CHANNEL_5;
//    ADC_Config.Rank         = 6;    	
//    ADC_Config.SamplingTime = ADC_SAMPLETIME_3CYCLES;// 采样时间间隔
//    ADC_Config.Offset       = 0;
//    HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config);

//   // 配置 ADC1 通道4转换顺序为7，采样时间为3个时钟周期，板上adc8
//    ADC_Config.Channel      = ADC_CHANNEL_4;
//    ADC_Config.Rank         = 7;    	
//    ADC_Config.SamplingTime = ADC_SAMPLETIME_3CYCLES;// 采样时间间隔
//    ADC_Config.Offset       = 0;
//    HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config);

//   // 配置 ADC1 通道13转换顺序为8，采样时间为3个时钟周期，板上adc7
//    ADC_Config.Channel      = ADC_CHANNEL_13;
//    ADC_Config.Rank         = 8;    	
//    ADC_Config.SamplingTime = ADC_SAMPLETIME_3CYCLES;// 采样时间间隔
//    ADC_Config.Offset       = 0;
//    HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config);
//   // 配置 ADC1 通道12转换顺序为9，采样时间为3个时钟周期，板上adc6
//    ADC_Config.Channel      = ADC_CHANNEL_12;
//    ADC_Config.Rank         = 9;    	
//    ADC_Config.SamplingTime = ADC_SAMPLETIME_3CYCLES;// 采样时间间隔
//    ADC_Config.Offset       = 0;
//    HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config);

//   // 配置 ADC1 通道10转换顺序为10，采样时间为3个时钟周期，板上adc5
//    ADC_Config.Channel      = ADC_CHANNEL_10;
//    ADC_Config.Rank         = 10;    	
//    ADC_Config.SamplingTime = ADC_SAMPLETIME_3CYCLES;// 采样时间间隔
//    ADC_Config.Offset       = 0;
//    HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config);

    HAL_ADC_Start_DMA(&ADC_Handle, (uint32_t*)&ADC_ConvertedValue,1);
	
	
}
void Rheostat_Init(void)
{
	Rheostat_ADC_GPIO_Config();
	Rheostat_ADC_Mode_Config();
}








/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
