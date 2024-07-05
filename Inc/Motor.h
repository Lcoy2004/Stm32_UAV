#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32h7xx_hal.h"
#define Motor_Vmin 1000
#define Motor_Vmax 2000

int8_t Motor_init(void);
int8_t Motor_update(double Motor_roll,double Motor_pitch,double Motor_height,double Motor_yaw );

void Motor_setspeed4(uint16_t speed);
void Motor_setspeed3(uint16_t speed);
void Motor_setspeed2(uint16_t speed);
void Motor_setspeed1(uint16_t speed);
#ifdef __cplusplus
}
#endif
#endif