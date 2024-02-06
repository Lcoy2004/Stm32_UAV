#include "stm32f10x.h"  
#include "imu.h"
#include "pid.h"
#include "Motor.h"
#include "chronoscope.h"
#include "myMath.h"
extern  T_gyro gyro;
extern T_angle angle;
extern uint16_t height;
extern const uint16_t Motor_Vmax;
extern const uint16_t Motor_Vmin;
//参数调试区
const PID_Calibration PID_yaw={0};
const PID_Calibration PID_pitch={0};
const PID_Calibration PID_roll={0};
const PID_Calibration PID_height={0};
const PID_Calibration PID_gyrox={0};
const PID_Calibration PID_gyroy={0};
const PID_Calibration PID_gyroz={0};
//
//
static PID_State PID_State_yaw;
static PID_State PID_State_pitch;
static PID_State PID_State_roll;
static PID_State PID_State_height;
static PID_State PID_State_gyroz;
static PID_State PID_State_gyroy;
static PID_State PID_State_gyrox;
//@para:传入的分别是目标数据target
void Control_Motor(float t_yaw,float t_pitch,float t_roll,uint16_t t_height)
{
double dt=0,Motor_roll,Motor_pitch,Motor_yaw,Motor_height;
dt=Clock2_End()*0.001;
//串级PID，姿态角为外环，角速度为内环,高度就一层PID
PID_State_height.target=(double)t_height;
PID_State_height.time_delta=(double)dt;
PID_State_height.actual=(double)height;
PID_State_height=pid_iterate(PID_height,PID_State_height);
Motor_height=PID_State_height.output;
//串级PID实现思路
//目标姿态角->外环姿态角PID->目标角速度->内环角速度PID->目标PWM
//对绕x轴的roll角进行PID
PID_State_roll.target=(double)t_roll;
PID_State_roll.actual=(double)angle.roll;
PID_State_roll.time_delta=(double)dt;
PID_State_roll=pid_iterate(PID_roll,PID_State_roll);//外环PID结束
PID_State_gyrox.target=PID_State_roll.output;//内环PID
PID_State_gyrox.actual=gyro.Gx;
PID_State_gyrox.time_delta=dt;
PID_State_gyrox=pid_iterate(PID_gyrox,PID_State_gyrox);
Motor_roll=PID_State_gyrox.output;
//对绕y轴的pitch角进行PID
PID_State_pitch.target=(double)t_pitch;
PID_State_pitch.actual=(double)angle.roll;
PID_State_pitch.time_delta=(double)dt;
PID_State_pitch=pid_iterate(PID_pitch,PID_State_pitch);//外环PID结束
PID_State_gyroy.target=PID_State_pitch.output;//内环PID
PID_State_gyroy.actual=gyro.Gy;
PID_State_gyroy.time_delta=dt;
PID_State_gyroy=pid_iterate(PID_gyroy,PID_State_gyroy);
Motor_pitch=PID_State_gyroy.output;
//对绕z轴的yaw进行PID（单纯只有6050其实实现意义不大）,yaw角逆时针为正
PID_State_yaw.target=(double)t_yaw;
PID_State_yaw.actual=(double)angle.yaw;
PID_State_yaw.time_delta=(double)dt;
PID_State_yaw=pid_iterate(PID_yaw,PID_State_yaw);//外环PID结束
PID_State_gyroz.target=PID_State_yaw.output;//内环PID
PID_State_gyroz.actual=gyro.Gy;
PID_State_gyroz.time_delta=dt;
PID_State_gyroz=pid_iterate(PID_gyroz,PID_State_gyroz);
Motor_yaw=PID_State_gyroz.output;

float motor1,motor2,motor3,motor4;
motor1=(float)(Motor_Vmin-Motor_roll-Motor_pitch+Motor_height+Motor_yaw);
motor2=(float)(Motor_Vmin-Motor_roll+Motor_pitch+Motor_height-Motor_yaw);
motor3=(float)(Motor_Vmin+Motor_roll-Motor_pitch+Motor_height-Motor_yaw);
motor4=(float)(Motor_Vmin+Motor_roll+Motor_pitch+Motor_height+Motor_yaw);
//防止超出
motor1=data_limit(motor1,(float)Motor_Vmax,(float)Motor_Vmin);
motor2=data_limit(motor2,(float)Motor_Vmax,(float)Motor_Vmin);
motor3=data_limit(motor3,(float)Motor_Vmax,(float)Motor_Vmin);
motor4=data_limit(motor4,(float)Motor_Vmax,(float)Motor_Vmin);
    Motor_SetSpeed1((uint16_t)(motor1));
	Motor_SetSpeed2((uint16_t)(motor2));
	Motor_SetSpeed3((uint16_t)(motor3));
	Motor_SetSpeed4((uint16_t)(motor4));
   Clock2_Start();
}