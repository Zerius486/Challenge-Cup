#ifndef __JL_GPIO_H__
#define __JL_GPIO_H__

#include "gpio.h"

#define GpioState(_x_)			((_x_##_GPIO_Port->IDR & _x_##_Pin)==_x_##_Pin?1:0)
#define GpioSet(_x_)			{_x_##_GPIO_Port->ODR |= _x_##_Pin;}
#define GpioReset(_x_)			{_x_##_GPIO_Port->ODR &= ~_x_##_Pin;}
#define GpioToggle(_x_)			{_x_##_GPIO_Port->ODR ^= _x_##_Pin;}







#endif /*__JL_GPIO_H__*/
