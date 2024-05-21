#ifndef __DATA_H__
#define __DATA_H__

#ifdef __cplusplus
extern "C" {
#endif

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
}T_coor;
/*定义数据变量*/
extern T_Acc Acc;
extern T_gyro Gyro;
extern T_angle Angle;
extern T_coor  flow;//光流传感

/*外部调用函数*/
int8_t Data_wit_Init();
int8_t Data_wit_Getimu();
#ifdef __cplusplus
}
#endif
#endif