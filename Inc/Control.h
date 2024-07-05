#ifndef __CONTROL_H__
#define __CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "pid.h"
#include "Data.h"
#include "stm32h7xx_hal.h"
/*参数调*/
static PID_Calibration PID_yaw;
static PID_Calibration PID_pitch; 
static PID_Calibration PID_roll;
static PID_Calibration PID_height;
static PID_Calibration PID_gyrox;
static PID_Calibration PID_gyroy;
static PID_Calibration PID_gyroz;
static PID_Calibration PID_ratex;
static PID_Calibration PID_ratey;
static PID_Calibration PID_coordx;
static PID_Calibration PID_coordy;

extern double Motor_roll,Motor_pitch,Motor_yaw,Motor_height;

int8_t Control_height_update(double t_height,double dt);
int8_t Control_coordinate_update(double t_coodx,double t_coody,double dt);
int8_t Control_attitude_update(double t_yaw,double t_roll,double t_pitch,double dt);
int8_t Control_pid_update(double t_height,double dt,T_angle target_angle,double t_coodx,double t_coody);
#ifdef __cplusplus
}
#endif
#endif