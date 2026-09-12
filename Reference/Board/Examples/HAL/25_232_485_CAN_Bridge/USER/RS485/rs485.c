#include "rs485.h"	
#include "string.h"
#include "stdlib.h"  
#include "led.h" 
#include "beep.h"
#include "change.h"

/*********************************************************************************
*********************启明欣欣 STM32F407应用开发板(高配版)*************************
**********************************************************************************
* 文件名称: rs485.c                                                              *
* 文件简述：rs485初始化                                                          *
* 创建日期：2017.08.30                                                           *
* 版    本：V1.0                                                                 *
* 作    者：Clever                                                               *
* 说    明：                                                                     * 
* 声    明：本例程代码仅用于学习参考                                             *
**********************************************************************************
*********************************************************************************/


  	  
//接收缓存区 	
u8 receive_str_485[128];   //接收缓冲,最大512个字节.
u8 send_str_485[128];
u8 byte_count_485=0;        //接收到的数据长度

u8 a485RxBuffer[RXBUFFERSIZE]; //HAL库使用的串口接收缓冲
UART_HandleTypeDef UART2_Handler; //UART句柄



/****************************************************************************
* 名    称: void HAL_UART_MspInit(UART_HandleTypeDef *huart)
* 功    能：UART底层初始化，时钟使能，引脚配置，中断配置
* 入口参数：huart:串口句柄
* 返回参数：无
* 说    明：此函数会被HAL_UART_Init()调用 
****************************************************************************/
//void HAL_UART_MspInit(UART_HandleTypeDef *huart)
//{
//    //GPIO端口设置
//	GPIO_InitTypeDef GPIO_Initure;
//	
//	if(huart->Instance==USART2)//如果是串口2，进行串口2 MSP初始化
//	{
//		__HAL_RCC_GPIOA_CLK_ENABLE();			//使能GPIOA时钟
//		__HAL_RCC_USART2_CLK_ENABLE();			//使能USART2时钟
//	
//		GPIO_Initure.Pin=GPIO_PIN_2;			//PA2
//		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
//		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
//		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//高速
//		GPIO_Initure.Alternate=GPIO_AF7_USART1;	//复用为USART1
//		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA9

//		GPIO_Initure.Pin=GPIO_PIN_3;			//PA3
//		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA10
//		
//		HAL_NVIC_EnableIRQ(USART2_IRQn);				//使能USART1中断通道
//		HAL_NVIC_SetPriority(USART2_IRQn,3,3);			//抢占优先级3，子优先级3	
//	}
//}

//初始化IO 串口2   bound:波特率	
void RS485_Init(u32 bound)
{  	 
  GPIO_InitTypeDef GPIO_InitStructure;
  __HAL_RCC_GPIOG_CLK_ENABLE();           //开启GPIOE时钟
	
		//PG6推挽输出，485模式控制  
	GPIO_InitStructure.Pin=GPIO_PIN_6; //PF9,10
	GPIO_InitStructure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
	GPIO_InitStructure.Pull=GPIO_PULLUP;          //上拉
	GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;     //高速
	HAL_GPIO_Init(GPIOG,&GPIO_InitStructure);
	
	RS485_TX_EN=0;				//初始化默认为接收模式	
	
  //UART 初始化设置
	UART2_Handler.Instance=USART2;					    //USART1
	UART2_Handler.Init.BaudRate=bound;				    //波特率
	UART2_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART2_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART2_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART2_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART2_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART2_Handler);					    //HAL_UART_Init()会使能UART1
	
	HAL_UART_Receive_IT(&UART2_Handler, (u8 *)a485RxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
}

//串口2接收中断服务函数
void USART2_IRQHandler(void)
{
	u8 rec_data;	  
  if((__HAL_UART_GET_FLAG(&UART2_Handler,UART_FLAG_RXNE)!=RESET))  //接收中断
	{	
     HAL_UART_Receive(&UART2_Handler,&rec_data,1,1000); 
		
		if(rec_data=='S')		  	                         //如果是S，表示是命令信息的起始位
				{
					byte_count_485=0x01; 
				}

			else if(rec_data=='E')		                         //如果E，表示是命令信息传送的结束位
				{
				 Change_485TO232(byte_count_485);        //当485接收到数据的时候从232原样发出去  启￥明#欣￥欣
					Change_485TOcan1(byte_count_485);       //当485接收到数据的时候从can1原样发出去BEEP=0; 	//蜂鸣器不响
					
					for(byte_count_485=0;byte_count_485<32;byte_count_485++)receive_str_485[byte_count_485]=0x00;
					byte_count_485=0;    
				}				  
			else if((byte_count_485>0)&&(byte_count_485<=USART2_REC_NUM))
				{
				   receive_str_485[byte_count_485-1]=rec_data;
				   byte_count_485++;
				}		
	}		
} 


/****************************************************************************
* 名    称: void RS485_Send_Data(u8 *buf,u8 len)
* 功    能：RS485发送len个字节
* 入口参数：buf:发送区首地址
            len:发送的字节数 
* 返回参数：无
* 说    明：(为了和本代码的接收匹配,这里建议数据长度不要超过512个字节)       
****************************************************************************/	
void RS485_Send_Data(u8 *buf,u8 len)
{
	RS485_TX_EN=1;			//设置为发送模式
	HAL_UART_Transmit(&UART2_Handler,buf,len,1000);//串口2发送数据
	RS485_TX_EN=0;				//设置为接收模式	
}







