#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx_hal.h"

#define FORCE_RS485_DE_Pin GPIO_PIN_0
#define FORCE_RS485_DE_GPIO_Port GPIOB
#define ETH_RESET_Pin GPIO_PIN_2
#define ETH_RESET_GPIO_Port GPIOE
#define LED0_Pin GPIO_PIN_3
#define LED0_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOG

void Error_Handler(void);

#endif
