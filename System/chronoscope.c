#include "stm32f10x.h"
void TIM1_Int_Init(void)
{

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE); 
  TIM_InternalClockConfig(TIM1);
  TIM_TimeBaseInitStructure.TIM_Period = 1000 -1; 
	TIM_TimeBaseInitStructure.TIM_Prescaler=72-1;  
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);
}
void Clock1_Start(){
	TIM1->CNT=0x00;
	TIM_Cmd(TIM1,ENABLE);
}

u16 Clock1_End(){
	u16 result;
	result = TIM1->CNT;
	TIM_Cmd(TIM1,DISABLE);
	return result;
}
void TIM4_Int_Init(void)
{

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE); 
  TIM_InternalClockConfig(TIM4);
  TIM_TimeBaseInitStructure.TIM_Period = 1000 -1; 
	TIM_TimeBaseInitStructure.TIM_Prescaler=72-1;  
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
}
void Clock2_Start(){
	TIM4->CNT=0x00;
	TIM_Cmd(TIM4,ENABLE);
}

u16 Clock2_End(){
	u16 result;
	result = TIM4->CNT;
	TIM_Cmd(TIM4,DISABLE);
	return result;
}