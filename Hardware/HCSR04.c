#include "stm32f10x.h"                  // Device header
#include "HCSR04.h"
#include "Timer.h"
#include "Delay.h"

uint16_t Time;

void HCSR04_Init()
{
	RCC_APB2PeriphClockCmd(Trig_RCC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = Trig_Pin;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(Trig_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStruct.GPIO_Pin = Echo_Pin;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(Echo_Port, &GPIO_InitStruct);
	
	GPIO_ResetBits(Trig_Port, Trig_Pin);
	
}

void HCSR04_Start()
{
	GPIO_SetBits(Trig_Port, Trig_Pin);
	Delay_us(45);
	GPIO_ResetBits(Trig_Port, Trig_Pin);
	Timer_Init();
}

uint16_t HCSR04_GetValue()
{
	HCSR04_Start();
	Delay_ms(100);
	return ((Time * 0.0001) * 34000) / 2;
//	return Time;
}
