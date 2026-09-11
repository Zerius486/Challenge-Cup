#ifndef __EX_LED_TASK_H__
#define __EX_LED_TASK_H__

#include "gpio.h"

#define ExLed1Pw_High()			{LedPw1_GPIO_Port->ODR |= LedPw1_Pin;}
#define ExLed1Pw_Low()			{LedPw1_GPIO_Port->ODR &= ~LedPw1_Pin;}
#define ExLed2Pw_High()			{LedPw2_GPIO_Port->ODR |= LedPw2_Pin;}
#define ExLed2Pw_Low()			{LedPw2_GPIO_Port->ODR &= ~LedPw2_Pin;}
#define ExLed3Pw_High()			{LedPw3_GPIO_Port->ODR |= LedPw3_Pin;}
#define ExLed3Pw_Low()			{LedPw3_GPIO_Port->ODR &= ~LedPw3_Pin;}
#define ExLed4Pw_High()			{LedPw4_GPIO_Port->ODR |= LedPw4_Pin;}
#define ExLed4Pw_Low()			{LedPw4_GPIO_Port->ODR &= ~LedPw4_Pin;}
#define ExLed5Pw_High()			{LedPw5_GPIO_Port->ODR |= LedPw5_Pin;}
#define ExLed5Pw_Low()			{LedPw5_GPIO_Port->ODR &= ~LedPw5_Pin;}

#define ExLedPwEnableCh1		ExLed1Pw_High()
#define ExLedPwDisableCh1		ExLed1Pw_Low()
#define ExLedPwEnableCh2		ExLed2Pw_High()
#define ExLedPwDisableCh2		ExLed2Pw_Low()
#define ExLedPwEnableCh3		ExLed3Pw_High()
#define ExLedPwDisableCh3		ExLed3Pw_Low()
#define ExLedPwEnableCh4		ExLed4Pw_High()
#define ExLedPwDisableCh4		ExLed4Pw_Low()
#define ExLedPwEnableCh5		ExLed5Pw_High()
#define ExLedPwDisableCh5		ExLed5Pw_Low()
#define ExLedPwEnableCh(x)		ExLedPwEnableCh##x
#define ExLedPwDisableCh(x)		ExLedPwDisableCh##x

#define ExLedDimCh1				2
#define ExLedDimCh2				8
#define ExLedDimCh3				7
#define ExLedDimCh4				6
#define ExLedDimCh5				5
#define ExLedDimCh(x)			ExLedDimCh##x
#endif /*__EX_LED_TASK_H__*/
