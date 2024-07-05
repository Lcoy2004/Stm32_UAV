#ifndef __DATA_H__
#define __DATA_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32h7xx_hal.h"
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

/*定义所有变量结构体*/
typedef struct 
{
double Ax;
double Ay;
double Az;
}T_Acc;

typedef struct 
{
double Gx;
double Gy;
double Gz;
}T_gyro;

typedef struct 
{
double roll;
double pitch;
double yaw;
}T_angle;

typedef struct 
{
double vx;
double vy;
}T_rate;
typedef struct 
{
double x;
double y;
}T_coor;
/*定义数据变量*/
/*以下是融合处理后的数，可以直接放入pid*/
extern T_Acc Acc;
 extern T_gyro Gyro;
 extern T_angle Angle;
 extern T_coor  Coor;//位移
 extern T_rate  Rate;//速度
 extern double height;//高度值
/*外部调用函数*/
int8_t Data_wit_Init();
int8_t Data_wit_Getimu();
int8_t Data_upixels_flowget(double dt,double dT);
int8_t Data_Height_fusion(uint8_t flag);
void State_modechange();
void State_stop();
void State_autofly();
void State_remotefly();
void State_monitering();//监视状态，临时改变状态;;
double Data_limit(double data, double toplimit, double lowerlimit);


#ifdef __cplusplus
}
#endif
#endif