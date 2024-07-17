#include "pid.h"
#include "Data.h"
#include "main.h"
#include "Control.h"
#include "State.h"
#include "Remote.h"
//姿态环与位置环的交换量
static double temp_pitch,temp_roll;
//得到的相应方向上的转速
double Motor_roll,Motor_pitch,Motor_yaw,Motor_height;
#define Gyro_pid_interg_limit 300
#define Angle_pid_interg_limit 200
#define Height_pid_interg_limit 300
#define Coor_pid_interg_limit 300
#define Rate_pid_interg_limit 200
//参数调试区
//绕X轴旋转角度为roll，绕Y轴旋转角度为pitch，绕Z轴旋转角度为yaw
PID_Calibration PID_yaw={0,0,0};//{3,0.05,0.025};
PID_Calibration PID_pitch={3.2,0.05,0.02}; //3.2,0.025,0.02
PID_Calibration PID_roll={3.2,0.05,0.02};//{2.51,0.04,0.01   3.95,0.17,0.03
PID_Calibration PID_gyrox={1.4,0.1,0.02};// {1.05,0.07,0.24}1.27,0.37,0.10
PID_Calibration PID_gyroy={1.4,0.1,0.02};// {1.95,0.025,0.025
PID_Calibration PID_gyroz={0,0,0};//{0.117,0.025,0.035}; 
PID_Calibration PID_ratex={2,0,0};
PID_Calibration PID_ratey={0,0,0};// {1.05,0.07,0.24}1.27,0.37,0.10
PID_Calibration PID_coordx={0,0,0};// {1.15,0.05,0.17}
PID_Calibration PID_coordy={0,0,0};//{1.10,0.1,0.13}; 
PID_Calibration PID_height={2,0,0};//{7.45,0,0};
static PID_State PID_State_yaw;
static PID_State PID_State_pitch;
static PID_State PID_State_roll;
static PID_State PID_State_height;
static PID_State PID_State_gyroz;
static PID_State PID_State_gyroy;
static PID_State PID_State_gyrox;
static PID_State PID_State_ratex;
static PID_State PID_State_ratey;
static PID_State PID_State_coordx;
static PID_State PID_State_coordy;


 int8_t Control_attitude_update(double t_yaw,double t_roll,double t_pitch,double dt)
 {
//串级姿态PID实现思路
//目标姿态角->外环姿态角PID->目标角速度->内环角速度PID->目标PWM
//对绕x轴的roll角进行PID
PID_State_roll.target=t_roll;
PID_State_roll.actual=Angle.roll;
PID_State_roll.time_delta=dt;
PID_State_roll=pid_iterate(PID_roll,PID_State_roll,Angle_pid_interg_limit);
//外环PID结束
PID_State_gyrox.target=PID_State_roll.output;//内环PID
PID_State_gyrox.actual=Gyro.Gx;
PID_State_gyrox.time_delta=dt;
PID_State_gyrox=pid_iterate(PID_gyrox,PID_State_gyrox,Gyro_pid_interg_limit);
Motor_roll=PID_State_gyrox.output;
//对绕y轴的pitch角进行PID
PID_State_pitch.target=t_pitch;
PID_State_pitch.actual=Angle.pitch;
PID_State_pitch.time_delta=dt;
PID_State_pitch=pid_iterate(PID_pitch,PID_State_pitch,Angle_pid_interg_limit);//外环PID结束
PID_State_gyroy.target=PID_State_pitch.output;//内环PID
PID_State_gyroy.actual=Gyro.Gy;
PID_State_gyroy.time_delta=dt;
PID_State_gyroy=pid_iterate(PID_gyroy,PID_State_gyroy,Gyro_pid_interg_limit);
Motor_pitch=PID_State_gyroy.output;
//对绕z轴的yaw进行PID,yaw角逆时针为正
PID_State_yaw.target=t_yaw;
PID_State_yaw.actual=Angle.yaw;
PID_State_yaw.time_delta=dt;
PID_State_yaw=pid_iterate(PID_yaw,PID_State_yaw,Angle_pid_interg_limit);//外环PID结束
PID_State_gyroz.target=PID_State_yaw.output;//内环PID
PID_State_gyroz.actual=Gyro.Gz;
PID_State_gyroz.time_delta=dt;
PID_State_gyroz=pid_iterate(PID_gyroz,PID_State_gyroz,Gyro_pid_interg_limit);
Motor_yaw=PID_State_gyroz.output;

    return UAVNormal;
 }

int8_t Control_coordinate_update(double t_coodx,double t_coody,double dt)
{
  /*这里是位置环pid*/
PID_State_coordx.target=t_coodx;
PID_State_coordx.actual=Coor.x;
PID_State_coordx.time_delta=dt;
PID_State_coordx=pid_iterate(PID_coordx,PID_State_coordx,Coor_pid_interg_limit);
//外环PID结束
PID_State_ratex.target=PID_State_coordx.output;//内环PID
PID_State_ratex.actual=Rate.vx;
PID_State_ratex.time_delta=dt;
PID_State_ratex=pid_iterate(PID_ratex,PID_State_ratex,Rate_pid_interg_limit);
temp_pitch=PID_State_ratex.output;//x轴速度环对应的是y轴的角度环！
double pid_y;
if (openmv_coody==0)
{
   pid_y=Coor.y;
}else
{
  pid_y=-openmv_coody;
}
PID_State_coordy.target=t_coody;
PID_State_coordy.actual=pid_y;
PID_State_coordy.time_delta=dt;
PID_State_coordy=pid_iterate(PID_coordy,PID_State_coordy,Coor_pid_interg_limit);
//外环PID结束
PID_State_ratey.target=PID_State_coordy.output;//内环PID
PID_State_ratey.actual=Rate.vy;
PID_State_ratey.time_delta=dt;
PID_State_ratey=pid_iterate(PID_ratey,PID_State_ratey,Rate_pid_interg_limit);
temp_roll=PID_State_ratey.output;//y轴速度环对应的是x轴的角度环！
return UAVNormal;
}

int8_t Control_height_update(double t_height,double dt)
{
   //测试值
   //t_height=200;
   //height=100;
   //
PID_State_height.target=(double)t_height;
PID_State_height.time_delta=(double)dt;
PID_State_height.actual=(double)height;
PID_State_height=pid_iterate(PID_height,PID_State_height,Height_pid_interg_limit);
Motor_height=PID_State_height.output;
return UAVNormal;
}
int8_t Control_pid_update(double t_height,double dt,T_angle target_angle,double t_coodx,double t_coody)
{
   static int8_t k;
   uint8_t loopk =8;
  if(current_state==UAVautofly)
  {
     if(k<loopk)
        {

          Control_attitude_update(target_angle.yaw,temp_roll,temp_pitch,dt);
          k++;
    
        }else{
           Control_coordinate_update(t_coodx,t_coody, loopk*dt);
          Control_attitude_update(target_angle.yaw,temp_roll,temp_pitch,dt);
           k=0;
        }
  }else if(current_state==UAVremotefly)
  {
     Control_attitude_update(target_angle.yaw,target_angle.roll,target_angle.pitch,dt);
  }
   Control_height_update(t_height,dt);
   return UAVNormal;
}

