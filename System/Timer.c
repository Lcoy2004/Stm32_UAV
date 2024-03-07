#include "stm32f10x.h"                  // Device header
#include "MPU6050.h"
#include "imu.h"
#include "Data.h"
#include "BMP280.h"
#include "Serial.h"
#include "Control.h"
#include "Receive.h"
extern T_angle P_angle;//存储角度测量值
extern float P_height;//存储测量高度值
extern float t_yaw;
extern float t_pitch;
extern float t_roll;
extern float t_height; 
extern float Rh;
void Timer2_Init()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//选择APB1总线下的定时器Timer2
	
	TIM_InternalClockConfig(TIM2);		//TIM2使用内部时钟
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;		//计数模式，此处为向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 72-1;		//ARR 1 = 0.0001S
	TIM_TimeBaseInitStructure.TIM_Prescaler = 10000-1;		//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;		//高级计时器特有，重复计数
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);		//使能中断
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;		//中断通道选择
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//优先级，同上
	
	NVIC_Init(&NVIC_InitStructure);
	
	//TIM_Cmd(TIM2, ENABLE);		//打开定时器
}
void Timer1_Init()
{

RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	TIM_InternalClockConfig(TIM1);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 72 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 5000 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
	
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStructure);
	//5ms
	TIM_Cmd(TIM1, ENABLE);
	
}
void Timer4_Init()
{
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	TIM_InternalClockConfig(TIM4);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 72 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1000 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
	//10ms
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM4, ENABLE);

}


void TIM1_UP_IRQHandler(void)//定时执行姿态解算
{
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		 MPU6050_Acc MA;
        MPU6050_gyro MG;
        //获取高度
    P_height=(BMP280_calculate_altitude()-Rh)*100.0f;
     MPU6050_GetData(&MA.Ax, &MA.Ay, &MA.Az, &MG.Gx, &MG.Gy,  &MG.Gz);
     //角度换算
     //Data_Calibrate(&MA,&MG,&P_height);//消除误差
    Acc_to_imu(MA.Ax,MA.Ay,MA.Az);
    Gyro_to_imu2(MG.Gx,MG.Gy,MG.Gz);
    imu_Getangle(MG,MA,&P_angle,0.005f);
	Serial_Printf("tim1\n");
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

void TIM2_IRQHandler()		//定时器2的中断函数,10ms进行pid
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
			
	Control_Motor(t_yaw,t_pitch,t_roll,t_height,0.01f);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
void TIM4_IRQHandler(void)//
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
			ReceiveNum_Gettarget();
		//Serial_Printf("tim4\n");
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}