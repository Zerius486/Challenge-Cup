#include "led.h"


/*********************************************************************************
**********************启明欣欣 STM32F407应用开发板(高配版)************************
**********************************************************************************
* 文件名称: 例程1 LED跑马灯主函数main()                                          *
* 文件简述：LED跑马灯                                                            *
* 创建日期：2018.10.03                                                           *
* 版    本：V1.0                                                                 *
* 作    者：Clever                                                               *
* 说    明：                                                                     * 
* 淘宝店铺：https://shop125046348.taobao.com                                     *
* 声    明：本例程代码仅用于学习参考                                             *
**********************************************************************************
*********************************************************************************/

/*******************下面代码是通过位带操作实现IO口控制***************************/
//int main(void)
//{ 
//  HAL_Init();                    	//初始化HAL库    
//  Stm32_Clock_Init(336,8,2,7);  	//设置时钟,168Mhz
//	delay_init();		  //初始化延时函数
//	LED_Init();		    //初始化LED端口
//	
//  while(1)
//	{
//    LED0=0;     //LED0亮
//    LED1=1;     //LED1灭
//    LED2=1;     //LED2灭
//		
//    delay_ms(500);
//		LED0=1;     //LED0灭
//    LED1=0;     //LED1亮
//    LED2=1;     //LED2灭
//		 
//		delay_ms(500);
//		LED0=1;     //LED0灭
//    LED1=1;     //LED1灭
//    LED2=0;     //LED2亮
//    delay_ms(500);
//	}
//}
/*********************************************************************************/

/*******************下面代码是通过位段操作实现IO口控制***************************/
	
int main(void)
{ 
  HAL_Init();                    	//初始化HAL库    
  Stm32_Clock_Init(336,8,2,7);  	//设置时钟,168Mhz
	delay_init();		  //初始化延时函数
	LED_Init();		    //初始化LED端口
	
  while(1)
	{
    GPIO_bits_OUT(GPIOE,3,2,0x0002);
	  delay_ms(500);
	  GPIO_bits_OUT(GPIOE,3,2,0x0001);
		delay_ms(500);
		GPIO_bits_OUT(GPIOE,3,2,0x0003);  //关闭LED0 LED1
	  GPIO_bits_OUT(GPIOG,9,1,0x0000);
		delay_ms(500);
		GPIO_bits_OUT(GPIOG,9,1,0x0001);  //关闭LED2 
	}
}
/***********************************************************************************/


/*******************下面代码是通过库函数直接操作实现IO口控制************************/

//int main(void)
//{ 
//  HAL_Init();                    	//初始化HAL库    
//  Stm32_Clock_Init(336,8,2,7);  	//设置时钟,168Mhz
//	delay_init();		  //初始化延时函数
//	LED_Init();		    //初始化LED端口
//	
//	while(1)
//	{
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET); 	//LED0对应引脚PE3拉低，亮，等同于LED0(0)
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);   	//LED1对应引脚PE4拉高，灭，等同于LED1(1)
//		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_9,GPIO_PIN_SET);   	//LED2对应引脚PG9拉高，灭，等同于LED2(1)
//		delay_ms(1000);											//延时500ms
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET); 	  //LED0对应引脚PE3拉低，灭，等同于LED0(1)
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET);   //LED1对应引脚PE4拉高，亮，等同于LED1(0)
//		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_9,GPIO_PIN_SET);   	//LED2对应引脚PG9拉高，灭，等同于LED2(1)
//		delay_ms(1000); 
//    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET); 	  //LED0对应引脚PE3拉低，灭，等同于LED0(1)
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);     //LED1对应引脚PE4拉高，灭，等同于LED1(1)
//		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_9,GPIO_PIN_RESET);   //LED2对应引脚PG9拉高，亮，等同于LED2(0)
//    delay_ms(1000); 		
//	}
//}





