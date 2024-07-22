/**
 * @file Motor.c
 * @author Lcoy (lcoy2004@qq.com)
 * @brief 实现电机控制
 * @version 1.0
 * @date 2024-07-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "Motor.h"
#include "Control.h"
#include "main.h"
#include "stm32h7xx_hal_tim.h"
#include "tim.h"
#include "Remote.h"
double data_limit1(double data, double toplimit, double lowerlimit);


int8_t Motor_update(double Motor_roll,double Motor_pitch,double Motor_height,double Motor_yaw )
{
double motor1,motor2,motor3,motor4;
motor1=(Motor_Vmin+data_limit1(power+Motor_height,850,0)+Motor_roll+Motor_pitch-Motor_yaw);
motor2=(Motor_Vmin+data_limit1(power+Motor_height,850,0)+Motor_roll-Motor_pitch+Motor_yaw);
motor3=(Motor_Vmin+data_limit1(power+Motor_height,850,0)-Motor_roll+Motor_pitch+Motor_yaw);
motor4=(Motor_Vmin+data_limit1(power+Motor_height,850,0)-Motor_roll-Motor_pitch-Motor_yaw);
//防止超出
motor1=data_limit1(motor1,Motor_Vmax,Motor_Vmin);
motor2=data_limit1(motor2,Motor_Vmax,Motor_Vmin);
motor3=data_limit1(motor3,Motor_Vmax,Motor_Vmin);
motor4=data_limit1(motor4,Motor_Vmax,Motor_Vmin);
/*下面motor输出*/
Motor_setspeed1((uint16_t)motor1);
Motor_setspeed2((uint16_t)motor2);
Motor_setspeed3((uint16_t)motor3);
Motor_setspeed4((uint16_t)motor4);

/*Motor End*/
return UAVNormal;

}

int8_t Motor_init(void)
{
HAL_TIM_Base_Start_IT(&htim2);
HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3); 
HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4); 
HAL_Delay(20);
Motor_setspeed1(Motor_Vmax);
Motor_setspeed2(Motor_Vmax);
Motor_setspeed3(Motor_Vmax);
Motor_setspeed4(Motor_Vmax);
HAL_Delay(1000);
Motor_setspeed1(Motor_Vmin);
Motor_setspeed2(Motor_Vmin);
Motor_setspeed3(Motor_Vmin);
Motor_setspeed4(Motor_Vmin);
HAL_Delay(2000);
Motor_setspeed1(Motor_Vmin);
Motor_setspeed2(Motor_Vmin);
Motor_setspeed3(Motor_Vmin);
Motor_setspeed4(Motor_Vmin);
return UAVNormal;
}

void Motor_setspeed1(uint16_t speed)
{
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
}

void Motor_setspeed2(uint16_t speed)
{
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);
}
void Motor_setspeed3(uint16_t speed)
{
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, speed);
}
void Motor_setspeed4(uint16_t speed)
{
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed);
}


 double data_limit1(double data, double toplimit, double lowerlimit)
{
  if(data > toplimit)  data = toplimit;
  else if(data < lowerlimit) data = lowerlimit;
    return data;
}