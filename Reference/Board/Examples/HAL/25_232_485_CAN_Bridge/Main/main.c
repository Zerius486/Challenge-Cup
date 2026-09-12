#include "led.h"
#include "beep.h"
#include "key.h"
#include "lcd.h"
#include "usart1.h"
#include "change.h"
#include "rs485.h"
#include "can1.h"
#include "can2.h"
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
		
	u8 cnt;
	
  HAL_Init();                    	//初始化HAL库    
  Stm32_Clock_Init(336,8,2,7);  	//设置时钟,168Mhz
	delay_init();     //延时函数初始化
	uart1_init(9600);	    //串口初始化波特率为9600
	RS485_Init(9600);
	LED_Init();		  		  //初始化与LED 
	BEEP_Init();          //蜂鸣器初始化
  LCD_Init();           //初始化LCD FSMC接口和显示驱动
	BRUSH_COLOR=RED;      //设置画笔颜色为红色
	
	LCD_DisplayString(10,10,24,"Illuminati STM32");	
  LCD_DisplayString(20,40,16,"Author:Clever");
	LCD_DisplayString(30,80,24,"5.DATA Change");
 
	/*232和485采用的是中断模式，故在中断接收函数中处理数据转换
	当接收到数据时，原样从485或232和can1发出去*/
  
	while(1)
	{		 
		 cnt=CAN1_Receive_Msg(receive_str_can1);
		 Change_can1TO232(cnt);
     Change_can1TO485(cnt);
		 delay_ms(10);
	}
}

