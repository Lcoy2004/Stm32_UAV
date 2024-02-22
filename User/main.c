#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Data.h"
#include "Motor.h"
#include "MPU6050.h"
#include "HCSR04.h"
#include "Serial.h"
#include "ESP.h"
#include "Control.h"
#include "Receive.h"
#include "Timer.h"
//先进行一些前期的调试工作
int main(void)
{
 //初始化各种硬件驱动	
 MPU6050_Init(); //MPU6050初始化
<<<<<<< HEAD
 //ZTW_Init();//电调及电机初始化
 ESP_Init();//ESP-12f初始化
=======
 ZTW_Init();//电调及电机初始化
 ESP_Init();//ESP-12f初始化
 
>>>>>>> 7488641d9f51e74d504d3cb4db6f0c2ca066bc71
 HCSR04_Init();//超声波初始化
 //Data_start();//消除零偏误差
 Serial_Init();   //串口初始化（调试用）
 Timer2_Init();
 Timer1_Init();
 Timer2_Init();
 //Timer4_Init();
while(1)
{
<<<<<<< HEAD
data_filter();//数据滤波得到相关值
//Data_pitch_SerialTest();
=======
data_filter();//数据滤波+处理得到相关值
Data_pitch_SerialTest();
>>>>>>> 7488641d9f51e74d504d3cb4db6f0c2ca066bc71
 //Data_row_SerialTest();
 //Data_yaw_SerialTest();
 //Data_Gyrox_SerialTest();
 //Data_Gyroy_SerialTest();
 //Data_Gyroz_SerialTest();
 //Data_height_SerialTest();
 //switch (ReceiveNum_Gettarget())
//{
//case 0: Control_stop();
 //   break;
//case 1: Control_fly();
  //  break;
//}
}
}
