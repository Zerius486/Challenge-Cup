#include "led.h"
#include "beep.h"
#include "key.h"
#include "lcd.h"
#include "can1.h"
#include "can2.h"
#include "usart1.h"
/*********************************************************************************
*********************启明欣欣 STM32F407应用开发板(高配版)*************************
**********************************************************************************
* 文件名称: 例程16 CAN实验                                                       *
* 文件简述：CAN实验                                                             *
* 创建日期：2020.08.30                                                           *
* 版    本：V1.0                                                                 *
* 作    者：Clever                                                               *
* 说    明：按键控制LED亮灭与蜂鸣器开断                                          *
* 淘宝店铺：https://shop125046348.taobao.com                                     *
* 声    明：本例程代码仅用于学习参考                                             *
**********************************************************************************
*********************************************************************************/

int main(void)
{ 
	u8 can1_sendbuf[8]="CAN1SEND";
	u8 can2_sendbuf[8]="CAN2SEND";
	
	u8 can1_RECbuf[8]={0};
	u8 can2_RECbuf[8]={0};
	
  HAL_Init();                    	//初始化HAL库    
  Stm32_Clock_Init(336,8,2,7);  	//设置时钟,168Mhz
	delay_init();     //延时函数初始化
  uart1_init(9600);
	LED_Init();				//LED初始化
	BEEP_Init();      //蜂鸣器初始化
	KEY_Init();       //按键初始化
 	LCD_Init();           //初始化LCD FSMC接口和显示驱动
	
	CAN1_Mode_Init(CAN_SJW_1TQ,CAN_BS2_6TQ,CAN_BS1_7TQ,6,CAN_MODE_NORMAL); //CAN初始化,波特率500Kbps     
	CAN1_Config();
  CAN2_Mode_Init(CAN_SJW_1TQ,CAN_BS2_6TQ,CAN_BS1_7TQ,6,CAN_MODE_NORMAL); //CAN初始化,波特率500Kbps     
	CAN2_Config();
  //则波特率为:42M/((1+6+7)*6)=500Kbps	
 	
 	BRUSH_COLOR=RED;//设置字体为红色 
	LCD_DisplayString(10,10,24,"Illuminati STM32");	
  LCD_DisplayString(10,40,16,"Author:Clever");
	LCD_DisplayString(30,60,24,"16.CAN TEST");

    CAN1_Send_Msg(can1_sendbuf,8);//发送8个字节   
	  delay_ms(10);
	  CAN2_Receive_Msg(can2_RECbuf); 
	  LCD_DisplayString(30,120,24,can2_RECbuf);   //液晶屏显示can2接收到的数值
	  uart1SendChars(can2_RECbuf,8);  //串口发出can2接收到的数值
  
/**************************************************************************************/

    CAN2_Send_Msg(can2_sendbuf,8);//发送8个字节   
    delay_ms(10);	
	  CAN1_Receive_Msg(can1_RECbuf); 
		LCD_DisplayString(30,180,24,can1_RECbuf); //液晶屏显示can1接收到的数值
	  uart1SendChars(can1_RECbuf,8);     //串口发出can1接收到的数值
 									  
while(1)
	{
		delay_ms(100);
		LED0=!LED0;//提示系统正在运行	   
	}
}

