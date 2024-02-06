#ifndef __HCSR04_H
#define __HCSR04_H
#include "stm32f10x.h"
#define Trig_Port 		GPIOA
#define Trig_Pin 		GPIO_Pin_0
#define Trig_RCC		RCC_APB2Periph_GPIOA

#define Echo_Port 		GPIOA
#define Echo_Pin 		GPIO_Pin_1
#define Echo_RCC		RCC_APB2Periph_GPIOA

void HCSR04_Init();
uint16_t HCSR04_GetValue();

#endif 