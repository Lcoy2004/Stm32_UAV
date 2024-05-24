#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32h7xx_hal.h"
#define Motor_Vmin 0
#define Motor_Vmax  1000

int8_t Motor_init(void);
int8_t Motor_update(double Motor_roll,double Motor_pitch,double Motor_height,double Motor_yaw );
#ifdef __cplusplus
}
#endif
#endif