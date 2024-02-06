#include "stm32f10x.h"                  // Device header
#include "Motor.h"
#include "Delay.h"
#include "chronoscope.h"
const uint16_t Motor_Vmax =1000;  //Vax: 设置电机转速为最大的值
const uint16_t Motor_Vmin =500;  //Vmin:设置电机转速为0时的值

void PWM_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	TIM_InternalClockConfig(TIM3);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;		//ARR.
	TIM_TimeBaseInitStructure.TIM_Prescaler = 144 - 1;		//PSC，每秒计数72M/144
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	 //给电调输入50hz
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;		//CCR
	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	
	TIM_Cmd(TIM3, ENABLE);
	
    
}

//speed大小值请看电调初始化的参数！
void Motor_SetSpeed1(uint16_t speed)//规定是正转电机 1
{
	TIM_SetCompare1(TIM3, speed);
}
 void Motor_SetSpeed2(uint16_t speed)//规定是反转电机1
{

	TIM_SetCompare2(TIM3, speed);
}
void Motor_SetSpeed3(uint16_t speed)//规定是正转电机2
{
	TIM_SetCompare3(TIM3, speed);
}
void Motor_SetSpeed4(uint16_t speed)//规定是反转电机2
{
	TIM_SetCompare4(TIM3, speed);
}
void ZTW_Init()//电调及电机——好盈 20A电调初始化
{
 PWM_Init();//开启电机驱动	
	Delay_ms(20);
   Motor_SetSpeed1(Motor_Vmax);
	Motor_SetSpeed2(Motor_Vmax);
	Motor_SetSpeed3(Motor_Vmax);
	Motor_SetSpeed4(Motor_Vmax);
		Delay_s(1);	
	Motor_SetSpeed1(Motor_Vmin);
	Motor_SetSpeed2(Motor_Vmin);
	Motor_SetSpeed3(Motor_Vmin);
	Motor_SetSpeed4(Motor_Vmin);
		Delay_s(3);
	//初始时刻关闭电机，防止意外
	Motor_SetSpeed1(Motor_Vmin);
	Motor_SetSpeed2(Motor_Vmin);
	Motor_SetSpeed3(Motor_Vmin);
	Motor_SetSpeed4(Motor_Vmin);
}