#include "led.h"
#include "beep.h"
#include "key.h"
#include "lcd.h"
#include "usart6.h"

/*********************************************************************************
*********************启明欣欣 STM32F407应用开发板(高配版)*************************
**********************************************************************************
* 文件名称: 例程3 按键使用主函数main()                                           *
* 文件简述：按键实验                                                             *
* 创建日期：2017.08.30                                                           *
* 版    本：V1.0                                                                 *
* 作    者：Clever                                                               *
* 说    明：按键控制LED亮灭与蜂鸣器开断                                          *
* 淘宝店铺：https://shop125046348.taobao.com                                     *
* 声    明：本例程代码仅用于学习参考                                             *
**********************************************************************************
*********************************************************************************/

int main(void)
{ 
  HAL_Init();                    	//初始化HAL库    
  Stm32_Clock_Init(336,8,2,7);  	//设置时钟,168Mhz
	delay_init();     //延时函数初始化
	uart6_init(9600);	    //串口初始化波特率为9600
	KEY_Init();
	LED_Init();		  		  //初始化与LED 
	BEEP_Init();          //蜂鸣器初始化
  LCD_Init();           //初始化LCD FSMC接口和显示驱动
	BRUSH_COLOR=RED;      //设置画笔颜色为红色
	
	LCD_DisplayString(10,10,24,"Illuminati STM32");	
  LCD_DisplayString(20,40,16,"Author:Clever");
	LCD_DisplayString(30,80,24,"5.USART6 TEST");
	LCD_DisplayString(50,110,16,"Please send control cmd");
  
	while(1)
	{
		key_scan(0);
		
		if(keyup_data==KEY0_DATA)
		  {
		    /*因printf()之类的函数，使用了半主机模式。使用标准库会导致程序无法
          运行,以下是解决方法:使用微库,因为使用微库的话,不会使用半主机模式. 
          请在工程属性的“Target“-》”Code Generation“中勾选”Use MicroLIB“这
          样以后就可以使用printf，sprintf函数了*/
				printf("串口打印测试\r\n");
				uart6SendChars("UART1 TEST\r\n",13);
		  }
	}
}

