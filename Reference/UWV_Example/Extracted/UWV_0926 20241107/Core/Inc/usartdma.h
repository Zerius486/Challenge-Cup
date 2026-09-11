#ifndef __usartdma_H
#define __usartdma_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
	 
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
 #define RCVDATA_SEND_BACK  //将接收的数据发送出去，测试用
#define BUF_SIZE 128
#define UART_BUF_SIZE 70
#define UART_BUF_SIZE3 66
#define UART_BUF_SIZE4 44	 
/* USER CODE END Includes */


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;	 	 
extern UART_HandleTypeDef huart6;
	 
	 
extern uint8_t rcvBuf[UART_BUF_SIZE];	 
extern uint8_t rcvBuf2[UART_BUF_SIZE];
extern uint8_t rcvBuf3[UART_BUF_SIZE3];
extern uint8_t rcvBuf4[UART_BUF_SIZE4];

/* USER CODE BEGIN Private defines */
void MX_DMA_Init(void);
/* USER CODE END Private defines */


void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_UART4_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void ProcessData(uint8_t uartId);
_Bool UartDmaSend(uint8_t uartid, uint8_t *buf, uint32_t len);
_Bool StartUartRxDMA(uint8_t uartId);
void HAL_UART_IdleCallback(UART_HandleTypeDef *huart);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif


#endif /*__JL_GPIO_H__*/
