#include "rs485.h"	
#include "string.h"
#include "stdlib.h"  
#include "led.h" 
#include "beep.h"
	 
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
u8 RS485_receive_str[512];   //接收缓冲,最大128个字节.
u8 uart_byte_count=0;        //接收到的数据长度

u8 aRxBuffer[RXBUFFERSIZE]; //HAL库使用的串口接收缓冲
UART_HandleTypeDef UART2_Handler; //UART句柄


//初始化IO 串口2   bound:波特率	
void RS485_Init(u32 bound)
{  	
    //GPIO端口设置
	GPIO_InitTypeDef GPIO_Initure;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();			//使能GPIOA时钟
	__HAL_RCC_GPIOG_CLK_ENABLE();			//使能GPIOG时钟	
	__HAL_RCC_USART2_CLK_ENABLE();			//使能USART2时钟
	
	GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3; //PA2,3
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
	GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;		//高速
	GPIO_Initure.Alternate=GPIO_AF7_USART2;	//复用为USART2
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA2,3
	
//PG6推挽输出，485模式控制  
	GPIO_Initure.Pin=GPIO_PIN_6; 			//PG6
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
	GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	HAL_GPIO_Init(GPIOG,&GPIO_Initure);
    
    //USART 初始化设置
	UART2_Handler.Instance=USART2;			        //USART2
	UART2_Handler.Init.BaudRate=bound;		        //波特率
	UART2_Handler.Init.WordLength=UART_WORDLENGTH_8B;	//字长为8位数据格式
	UART2_Handler.Init.StopBits=UART_STOPBITS_1;		//一个停止位
	UART2_Handler.Init.Parity=UART_PARITY_NONE;		//无奇偶校验位
	UART2_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;	//无硬件流控
	UART2_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART2_Handler);			        //HAL_UART_Init()会使能USART2
    
  __HAL_UART_DISABLE_IT(&UART2_Handler,UART_IT_TC);
	__HAL_UART_ENABLE_IT(&UART2_Handler,UART_IT_RXNE);//开启接收中断
	HAL_NVIC_EnableIRQ(USART2_IRQn);				        //使能USART1中断
	HAL_NVIC_SetPriority(USART2_IRQn,3,3);			        //抢占优先级3，子优先级3

	RS485_TX_EN=0;											//默认为接收模式
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
					uart_byte_count=0x01; 
				}

			else if(rec_data=='E')		                         //如果E，表示是命令信息传送的结束位
				{
					if(strcmp("Light_led1",(char *)RS485_receive_str)==0)        LED1=0;	//点亮LED1
					else if(strcmp("Close_led1",(char *)RS485_receive_str)==0)   LED1=1;	//关灭LED1
					else if(strcmp("Open_beep",(char *)RS485_receive_str)==0)    BEEP=1; 	//蜂鸣器响
					else if(strcmp("Close_beep",(char *)RS485_receive_str)==0)   BEEP=0; 	//蜂鸣器不响
					
					for(uart_byte_count=0;uart_byte_count<32;uart_byte_count++)RS485_receive_str[uart_byte_count]=0x00;
					uart_byte_count=0;    
				}				  
			else if((uart_byte_count>0)&&(uart_byte_count<=USART2_REC_NUM))
				{
				   RS485_receive_str[uart_byte_count-1]=rec_data;
				   uart_byte_count++;
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







