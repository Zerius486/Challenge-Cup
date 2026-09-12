#include "led.h"
#include "beep.h"
#include "key.h"
#include "lcd.h"
#include "rs485.h"

/*********************************************************************************
*********************启明欣欣 STM32F407应用开发板(高配版)*************************
**********************************************************************************
* 文件名称: 例程12 串口2-485实验                                                 *
* 文件简述：485实验                                                              *
* 创建日期：2020.08.30                                                           *
* 版    本：V1.0                                                                 *
* 作    者：Clever                                                               *
* 说    明：串口命令控制LED亮灭与蜂鸣器开断                                      *
* 淘宝店铺：https://shop125046348.taobao.com                                     *
* 声    明：本例程代码仅用于学习参考                                             *
**********************************************************************************
*********************************************************************************/

int main(void)
{ 
  HAL_Init();                    	//初始化HAL库    
  Stm32_Clock_Init(336,8,2,7);  	//设置时钟,168Mhz
	delay_init();     //延时函数初始化
	RS485_Init(9600);	    //串口初始化波特率为9600
	KEY_Init();
	LED_Init();		  		  //初始化与LED 
	BEEP_Init();          //蜂鸣器初始化
  LCD_Init();           //初始化LCD FSMC接口和显示驱动
 	BRUSH_COLOR=RED;    //设置字体为红色 
	LCD_DisplayString(10,10,24,"Illuminati STM32");	
  LCD_DisplayString(20,40,16,"Author:Clever");
	LCD_DisplayString(30,80,24,"12.USART2 TEST");	
	LCD_DisplayString(30,130,16,"KEY0:Send");    	//显示提示信息		
 									  
	while(1)
	{
		key_scan(0);
		
		if(keyup_data==KEY0_DATA)
		  {
		   RS485_Send_Data("UART2 TEST",11);
		  }
	}
}

