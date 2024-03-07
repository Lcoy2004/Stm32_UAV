#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Data.h"
#include "Motor.h"
#include "MPU6050.h"
#include "BMP280.h"
#include "Serial.h"
#include "ESP.h"
#include "Control.h"
#include "Receive.h"
#include "Timer.h"
//先进行一些前期的调试工作
int main(void)
{
 //初始化各种硬件驱动	
 ZTW_Init();//电调及电机初始化，最好放在前面
 MPU6050_Init(); //MPU6050初始化
 ESP_Init();//ESP-12f初始化
 Bmp_Init();//气压计初始化
// Data_start();//消除零偏误差
 Data_Height_Calibrate();//得到初始高度
 Serial_Init();   //串口初始化（调试用）
 Timer2_Init();
 Timer1_Init();
 Timer4_Init();
 
 //TIM_Cmd(TIM2, ENABLE);//开启pid,调参完后请注释掉
while(1)
{
//ReceiveNum_Gettarget();
data_filter();//数据滤波得到相关值
//Data_t_pitch_SerialTest();
Data_t_height_SerialTest();
//Data_angle_SerialTest();
//Data_pitch_SerialTest();
 //Data_row_SerialTest();
 //Data_yaw_SerialTest();
 //Data_Gyrox_SerialTest();
 //Data_Gyroy_SerialTest();
 //Data_Gyroz_SerialTest();
 //Data_height_SerialTest();
// switch (ReceiveNum_Gettarget())
//{
//case 0: Control_stop();
   //break;
//case 1: Control_fly();
    //break;
//}
}
}
