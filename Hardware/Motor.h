#ifndef  __MOTOR_H__
#define __MOTOR_H__
void PWM_Init(void);
void Motor_SetSpeed1(uint16_t speed); //规定是正转电机 1黑
void Motor_SetSpeed2(uint16_t speed);  //规定是反转电机 1黑
void Motor_SetSpeed3(uint16_t speed);  //规定是正转电机 2白
void Motor_SetSpeed4(uint16_t speed);  //规定是反转电机 2白
uint16_t Motor_GetSpeed1();//规定是正转电机 1
 uint16_t Motor_GetSpeed2();//规定是反转电机1
uint16_t Motor_GetSpeed3();//规定是正转电机2
uint16_t Motor_GetSpeed4();//规定是反转电机2
void ZTW_Init();//电调及电机——好盈 20A电调初始化
 //这里使用了tim3，占用端口GPIOB0，1；GPIOA6，7；用于输出pwm
 //相应正转和反转电机已设置好
 //@par:speed:大小值请看电调初始化的参数！
#endif
