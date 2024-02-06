#ifndef  __IMU_H__
#define __IMU_H__
#include "stm32f10x.h"
typedef  volatile struct 
{
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;
typedef struct 
{
float Ax;
float Ay;
float Az;
}T_Acc;

typedef struct 
{
float Gx;
float Gy;
float Gz;
}T_gyro;

typedef struct 
{
float roll;
float yaw;
float pitch;
}T_angle;

typedef struct 
{
int16_t Ax;
int16_t Ay;
int16_t Az;
}MPU6050_Acc;

typedef struct 
{
int16_t Gx;
int16_t Gy;
int16_t Gz;
}MPU6050_gyro;
void imu_Getangle(MPU6050_gyro gyro,MPU6050_Acc Acc,T_angle *angle,float dt);
#endif
