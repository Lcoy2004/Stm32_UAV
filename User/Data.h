#ifndef  __DATA_H__
#define __DATA_H__
#include "Kalman.h"
#include "imu.h"
#include "stm32f10x.h"
void data_filter(void);//数据整合
void Data_start();//初始校准，消除零偏误差
//串口测试用
void Data_pitch_SerialTest();
void Data_row_SerialTest();
void Data_yaw_SerialTest();
void Data_Gyrox_SerialTest();
void Data_Gyroy_SerialTest();
void Data_Gyroz_SerialTest();
void Data_height_SerialTest();
///void Gyro_to_imu2(int16_t Gx,int16_t Gy,int16_t Gz);//角速度换算弧度
//void Acc_to_imu(int16_t Ax,int16_t Ay,int16_t Az);//MPU6050加速度换算

 
extern T_Acc P_Acc;//存储加速度值测量值
extern T_gyro gyro;//存储滤波后角速度值
extern T_angle angle;//存储滤波后姿态角值
extern uint16_t height;//存储滤波后高度值
extern uint16_t P_height;//存储测量高度值
extern T_gyro P_gyro;//存储角速度测量值
extern T_angle P_angle;//存储角度测量值






#endif