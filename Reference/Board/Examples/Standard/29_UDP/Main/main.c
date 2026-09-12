#include "led.h"
#include "lcd.h"
#include "key.h"
#include "led.h"
#include "key.h"
#include "lwip_base.h" 
#include "lan8720.h"
#include "timer.h"
#include "lcd.h"
#include "usart1.h"
//#include "sram.h"
#include "malloc.h"
#include "lwip/netif.h"
#include "lwipopts.h"
#include "udp_demo.h"
#include "string.h" 

///*********************************************************************************
//******************启明欣欣 STM32F407应用开发板(高配版)****************************
//**********************************************************************************
//* 文件名称: 例程25 UDP网络实验                                                   *
//* 文件简述：DP83848_LWIP实验                                                     *
//* 创建日期：2015.010.05                                                          *
//* 版    本：V1.0                                                                 *
//* 作    者：Clever                                                               *
//* 说    明：完成TCP的客户端的数据收发                                            * 
//* 淘宝店铺：https://shop125046348.taobao.com                                     *
//* 声    明：本例程代码仅用于学习参考                                             *
//**********************************************************************************
//*********************************************************************************/

	
	
int main(void)
{	
//  u8 t;
	u8 speed;
	u8 buf[30]; 
	
	delay_init();       	//延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart1_init(115200);   	//串口波特率设置
	
	LED_Init();  			//LED初始化
	KEY_Init();  			//按键初始化
	LCD_Init();
//  FSMC_SRAM_Init();		//初始化外部SRAM  
	
	TIM3_Int_Init(999,839); //100khz的频率,计数1000为10ms
	
	my_mem_init(SRAMIN);		//初始化内部内存池
//  my_mem_init(SRAMEX);		//初始化外部内存池

	BRUSH_COLOR = RED; 		//红色字体
		
	LCD_DisplayString(10,10,24,"Illuminati STM32");	
  LCD_DisplayString(10,40,16,"Author:Clever");
	LCD_DisplayString(20,70,24,"25 UDP_TEST ");
	
	
//先初始化lwIP(包括DP83848初始化),此时必须插上网线,否则初始化会失败!! 
	LCD_DisplayString(20,110,16,"lwIP Initing...");
	while(lwip_comm_init()!=0)
	{
		LCD_DisplayString(20,110,16,"lwIP Init failed!");
		delay_ms(1200);
		LCD_Fill_onecolor(20,110,230,110+16,WHITE);//清除显示
		LCD_DisplayString(20,110,16,"Retrying...");  
	}
	LCD_DisplayString(20,110,16,"lwIP Init Successed");
	
	//等待DHCP获取 
 	LCD_DisplayString(20,130,16,"DHCP IP configing...");
	while((lwipdev.dhcpstatus!=2)&&(lwipdev.dhcpstatus!=0XFF))//等待DHCP获取成功/超时溢出
	{
		lwip_periodic_handle();
	}
	
 //DHCP获取后显示本地IP地址(开发板IP地址) 	
	LCD_Fill_onecolor(20,110,lcd_width,lcd_height,WHITE);	//清除显示
	LCD_DisplayString(20,110,16,"lwIP Init Successed");
	if(lwipdev.dhcpstatus==2)sprintf((char*)buf,"DHCP IP:%d.%d.%d.%d",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]); //DHCP成功后打印动态IP地址
		else sprintf((char*)buf,"Static IP:%d.%d.%d.%d",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);//DHCP失败后打印静态IP地址
	LCD_DisplayString(20,130,16,buf); //屏幕显示开发板IP地址
	
	speed=LAN8720_Get_Speed();//得到网速
	if(speed&1<<1)LCD_DisplayString(20,150,16,"Ethernet Speed:100M");
		else LCD_DisplayString(20,150,16,"Ethernet Speed:10M");
	
	
	while(1)
	{
	 udp_demo_test();
  }
   	
}


