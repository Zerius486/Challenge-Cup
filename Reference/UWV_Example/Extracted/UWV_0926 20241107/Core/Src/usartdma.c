/* Includes ------------------------------------------------------------------*/
#include "usartdma.h"

#include "stm32f4xx_hal_def.h"
#include "string.h"

/*
1、STM32F429有2个DMA控制器(DMA1和DMA2),每个控制器有8个数据流，每个数据流有8个通道(请求)。
2、支持外设到存储器、存储器到外设、存储器到存储器传输的常规通道。(仅DMA2支持存储器到存储器的传输)
//Author：zcr 7_9
*/

//usart1： rx pa9，tx pa8
//usart2： rx pd6，tx pd5
//usart3： rx pd9，tx pd8
//uart4： rx pc11，tx pc10
//uart5： rx pd2，tx pc12
/* USER CODE BEGIN 0 */
typedef uint8_t bool;
#define TRUE 1
#define FALSE 0
uint8_t gUartBuf[BUF_SIZE] = {0};
uint8_t sndBuf[UART_BUF_SIZE] = {0};
uint8_t rcvBuf[UART_BUF_SIZE] = {0};
uint8_t sndBuf2[UART_BUF_SIZE] = {0};
uint8_t rcvBuf2[UART_BUF_SIZE] = {0};

uint8_t sndBuf3[UART_BUF_SIZE3] = {0};
uint8_t rcvBuf3[UART_BUF_SIZE3] = {0};

uint8_t sndBuf4[UART_BUF_SIZE4] = {0};
uint8_t rcvBuf4[UART_BUF_SIZE4] = {0};

uint8_t sndBuf6[UART_BUF_SIZE] = {0};
uint8_t rcvBuf6[UART_BUF_SIZE] = {0};

#ifdef RCVDATA_SEND_BACK
bool sendCompleteSign = TRUE,sendCompleteSign2 = TRUE,sendCompleteSign3 = TRUE,sendCompleteSign4 = TRUE,sendCompleteSign6 = TRUE;
#endif

/* USER CODE END 0 */



UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart6;

DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;


DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

DMA_HandleTypeDef hdma_usart4_rx;
DMA_HandleTypeDef hdma_usart4_tx;

DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;
DMA_InitTypeDef   tt8kkl;

//static void DMA_Conf(void)
//{
//    DMA_InitTypeDef DMA_Structure;
//	DMA_HandleTypeDef dma_rx2;

// 
//  /* DMA controller clock enable */
//  __HAL_RCC_DMA2_CLK_ENABLE();
//  __HAL_RCC_DMA1_CLK_ENABLE();
//	
//    DMA_DeInit(DMA1_Stream1);
//    DMA_DeInit(DMA1_Stream3);
//    while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE){}
//    while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}
//    /*配置串口2接收流 */
//     dma_rx2.Instance = DMA1_Stream5;  
//	 dma_rx2.Instance->PAR=(uint32_t)(&USART3->DR);
//	 dma_rx2.Instance->M0AR=(uint32_t)&rcvBuf2;
//	 dma_rx2.Init.Channel = DMA_CHANNEL_4;                    /*DMA1通道4*/
//    
//		
//	dma_rx2.Init.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
//    dma_rx2.Init.DMA_Memory0BaseAddr = (uint32_t)dma_rx_buf;
//    dma_rx2.Init.DMA_DIR = DMA_DIR_PeripheralToMemory;           /*外设到内存*/
//    dma_rx2.Init.DMA_BufferSize = sizeof(dma_rx_buf);
//    dma_rx2.Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//    dma_rx2.Init.DMA_MemoryInc = DMA_MemoryInc_Enable;
//    dma_rx2.Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//    dma_rx2.Init.MemoryDataSize = DMA_MemoryDataSize_Byte;
//    dma_rx2.Init.Mode = DMA_CIRCULAR;                   /*循环模式*/
//    dma_rx2.Init.Priority = DMA_Priority_Low;
//    dma_rx2.Init.DMA_FIFOMode = DMA_FIFOMode_Disable;         
//    dma_rx2.Init.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
//    dma_rx2.Init.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//    dma_rx2.Init.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//    DMA_Init(DMA1_Stream1, &DMA_Structure); 
//    dma_rx2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//   
//   if (HAL_DMA_Init(&dma_rx2) != HAL_OK)
//    {
//      Error_Handler();
//    }
//	
//    /*配置串口3发送流 */
//    DMA_Structure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
//    DMA_Structure.DMA_Memory0BaseAddr = (uint32_t)dma_tx_buf;
//    DMA_Structure.DMA_DIR = DMA_DIR_MemoryToPeripheral;            /*内存到外设*/
//    DMA_Structure.DMA_BufferSize = sizeof(dma_tx_buf);
//    DMA_Structure.DMA_Mode = DMA_Mode_Normal;                      /*正常模式 -*/
//    DMA_Init(DMA1_Stream3, &DMA_Structure); 
// 
//    /* Enable DMA Stream Transfer Complete interrupt */
//    DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);    
//    //DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
//    /* DMA Stream enable */
//    DMA_Cmd(DMA1_Stream1, ENABLE);                                 /*使能接收流*/
// 
//    /* Enable the DMA Stream IRQ Channel */
//    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);  
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
//    NVIC_Init(&NVIC_InitStructure);  
//}

void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 3); //hdma_usart3_rx channel-4
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 3); //hdma_usart3_tx channel-4
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0); //hdma_usart2_rx
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0); //hdma_usart2_tx
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 2); //hdma_uart4_tx channel-4
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 2); //hdma_uart4_rx channel-4
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);	
	
	
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 3, 2); //hdma_usart6_rx
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);//hdma_usart1_rx
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 3, 2); //hdma_usart6_tx
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  
  /* DMA2_Stream7_IRQn interrupt configuration *///hdma_usart1_tx
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 256000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 230400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* UART4 init function */
void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 460800;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }

}


void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PA0-WKUP     ------> UART4_TX  pc10
    PA1     ------> UART4_RX       pc11
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


  /* USER CODE BEGIN UART4_MspInit 1 */
    /* UART4 DMA Init */
    /* UART4_RX Init */
    hdma_usart4_rx.Instance = DMA1_Stream2;
    hdma_usart4_rx.Init.Channel = DMA_CHANNEL_4;	
    hdma_usart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart4_rx.Init.MemInc = DMA_MINC_ENABLE;	
    hdma_usart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart4_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart4_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_usart4_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_usart4_rx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_usart4_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;	
    if (HAL_DMA_Init(&hdma_usart4_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart4_rx);

    /* USART4_TX Init */
    hdma_usart4_tx.Instance = DMA1_Stream4;
    hdma_usart4_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart4_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart4_tx.Init.Mode = DMA_NORMAL;
    hdma_usart4_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart4_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart4_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart4_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 0, 3);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* USER CODE END UART4_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream5;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//    hdma_usart1_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
//    hdma_usart1_rx.Init.MemBurst = DMA_MBURST_SINGLE;
//    hdma_usart1_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;			
		
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }


		
		
		
		
    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init DMA_CIRCULAR DMA_NORMAL*/
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;	
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_usart2_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_usart2_rx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_usart2_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;	
	
	
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART2_TX Init */
    hdma_usart2_tx.Instance = DMA1_Stream6;
    hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart2_tx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;	
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;	
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_usart3_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_usart3_rx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_usart3_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;		
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3_TX Init */
    hdma_usart3_tx.Instance = DMA1_Stream3;
    hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart3_tx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART6 DMA Init */
    /* USART6_RX Init */
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_NORMAL;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart6_rx);

    /* USART6_TX Init */
    hdma_usart6_tx.Instance = DMA2_Stream6;
    hdma_usart6_tx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_tx.Init.Mode = DMA_NORMAL;
    hdma_usart6_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart6_tx);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 3, 2);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PA0-WKUP     ------> UART4_TX
    PA1     ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);

    /* UART4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5|GPIO_PIN_6);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_9);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

    /* USART6 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
uint8_t * msg = rcvBuf;
int rcvLen=0;
uint8_t testFlag = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//全满中断
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
	if(huart->Instance == USART1)
    {
        ProcessData(1);
        StartUartRxDMA(1); //restart usart1 dma receive
    }
	else  if(huart->Instance == USART2)
    {
      //  ProcessData(2);
		
    //   StartUartRxDMA(2); //restart usart2 dma receive
    }
	else if(huart->Instance == USART3)
    {
//        ProcessData(3);
//        StartUartRxDMA(3); //restart usart3 dma receive
    }
	else if(huart->Instance == UART4)
    {
//        ProcessData(4);
//        StartUartRxDMA(4); //restart usart3 dma receive
    }	
	
	
	else  if(huart->Instance == USART6)
    {
        ProcessData(6);
        StartUartRxDMA(6); //restart usart6 dma receive
    }

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
#ifdef RCVDATA_SEND_BACK
    if(huart == &huart1)
    {
        sendCompleteSign = TRUE;
    }
    else if(huart == &huart2)
    {
        sendCompleteSign2 = TRUE;
    }
    else if(huart == &huart3)
    {
        sendCompleteSign3 = TRUE;
    }
    else if(huart == &huart6)
    {
        sendCompleteSign6 = TRUE;
    }
#endif
}

uint16_t recnt=0;

void HAL_UART_IdleCallback(UART_HandleTypeDef *huart)//空闲中断
{
    //uint32_t len = 0;
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	{
		HAL_UART_DMAStop(huart);	
		if(huart->Instance == USART1)
		{
			ProcessData(1);
			StartUartRxDMA(1);
		}
		if(huart->Instance == USART2)
		{
		//	ProcessData(2);
		recnt++;
		//	StartUartRxDMA(2);
		}
		else if(huart->Instance == USART3)
	{
//			ProcessData(3);
			//StartUartRxDMA(3);
		}
		else if(huart->Instance == UART4)
		{
//			ProcessData(4);
		//	StartUartRxDMA(4);
		}		
		
		else if(huart->Instance == USART6)
		{
			ProcessData(6);
			StartUartRxDMA(6);
		}
	}
}

void ProcessData(uint8_t uartId)
{
    uint32_t len = 0;

    switch(uartId)
    {
    	case 1:
    		//已经接收的字节 = 总共要接收的字节数 - NDTR
			len = UART_BUF_SIZE - huart1.hdmarx->Instance->NDTR;
			break;
			
    	case 2:
			len = UART_BUF_SIZE - huart2.hdmarx->Instance->NDTR;
			break;
			
    	case 4:
			len = UART_BUF_SIZE - huart4.hdmarx->Instance->NDTR;//
			break;
			
    	case 3:
			len = UART_BUF_SIZE - huart3.hdmarx->Instance->NDTR;//
			break;			
			
    	case 6:
			len = UART_BUF_SIZE - huart6.hdmarx->Instance->NDTR;
			break;

    	default:
    		len = 0;
    		break;
    }

}

_Bool UartRxData(UART_HandleTypeDef *huart, uint8_t *buf, uint32_t len)
{
	HAL_StatusTypeDef status;
	bool ret = TRUE;

	status = HAL_UART_Receive_DMA(huart, (uint8_t*)buf, len);
	if(HAL_OK != status)
	{
		ret = FALSE;
	}
	else
	{
		/* 开启空闲接收中断 */
	 //   __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
		
		
	}

	return ret;
}

//启动DMA接收
_Bool StartUartRxDMA(uint8_t uartId)
{
	switch(uartId)
	{
		case 1:
			UartRxData(&huart1, rcvBuf, UART_BUF_SIZE);
			break;
		case 2:
			UartRxData(&huart2, rcvBuf2, UART_BUF_SIZE);
			break;
		case 3:
			UartRxData(&huart3, rcvBuf3, UART_BUF_SIZE3);
			break;
		case 4:
			UartRxData(&huart4, rcvBuf4, UART_BUF_SIZE4);
			break;	
		case 6:
			UartRxData(&huart6, rcvBuf6, UART_BUF_SIZE);
			break;
		default:
			break;
	}
    return 1;
}

_Bool UartDmaSend(uint8_t uartid, uint8_t *buf, uint32_t len)
{
	HAL_StatusTypeDef status;
	_Bool ret = TRUE;
	if(len == 0)
		return 0;
	switch(uartid)
	{
		case 1:
			status = HAL_UART_Transmit_DMA(&huart1, (uint8_t*)buf, len);
			break;
		case 2:
			status = HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buf, len);
			break;
		case 3:
			status = HAL_UART_Transmit_DMA(&huart3, (uint8_t*)buf, len);
			break;
		case 4:
			status = HAL_UART_Transmit_DMA(&huart4, (uint8_t*)buf, len);
			break;		
		
		case 6:
			status = HAL_UART_Transmit_DMA(&huart6, (uint8_t*)buf, len);
			break;
		default:
			break;
	}

	if(HAL_OK != status)
	{
		ret = FALSE;
	}

	//StartUartRxDMA(uartid);

	return ret;
}









