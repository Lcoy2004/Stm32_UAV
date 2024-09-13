#include "filter.h"
#include "Data.h"
#include "stdio.h"
#include "main.h"
#include "CalculateFlow.h"
#include "Kalman.h"
static K_Filter K_Ratex={0,0.02f,0.15f,0.02f,0,0};//需要调参
static K_Filter K_Ratey={0,0.02f,0.15f,0.02f,0,0};//需要调参
int16_t flow_x_integral = 0;
 int16_t flow_y_integral = 0;
 uint16_t ground_distance ;
uint8_t valid ;
 uint8_t tof_confidence ;

/*传感器数据*/
 T_coor  flow_Coor;//光流积分得到的值
 T_rate  flow_Rate;//光流得到的速度值
_filter_1_st f1_fx;
_filter_1_st f1_fy;
 //dt:光流采样；dT：imu采样
//优象光流的xy轴与imu不太一致。若对齐x轴，两个y轴相反，因此这里默认imux轴为飞机y轴，光流的y轴取正
void CalculateFlow_upixels_Complementary(double dt,T_gyro gyro)
{
static double speed_err_ix,speed_err_iy;
double flow_gyrox,flow_gyroy;
double Temp_vx,Temp_vy;
//T_rate com_rate融合补偿后的速度
		   flow_gyroy=(double)flow_x_integral/10000/dt;//rad/s，绕y轴的光流角速度
		   flow_gyrox=(double)flow_y_integral/10000/dt;//rad/s，绕x轴的光流角速度
        flow_Coor.x+=((double)flow_x_integral/10000)*(height);//累加位移cm
		   flow_Coor.y+=(double)flow_y_integral/10000*(height);//累加位移cm
      // printf("Flow_coor:%lf,%lf\n", flow_Coor.x,flow_Coor.y);

          // flow_Rate.vx=(double)flow_x_integral/10000*(height)/dt;//cm/s
		   //flow_Rate.vy=(double)flow_y_integral/10000*(height)/dt;//cm/s
    double filter_gyrox,filter_gyroy;
      //融合补偿
     filter_gyrox=flow_gyrox-Filter_limit((gyro.Gx/57.3),1.0,-1.0);//低通滤波后gyro有衰减
     filter_gyroy=flow_gyroy+Filter_limit((gyro.Gy/57.3),1.0,-1.0);
   //printf("flowgyro:%lf,%lf,%lf\n",flow_gyrox,gyrox_imufilter,filter_gyrox);
   //printf("flowgyro:%lf,%lf,%lf\n",flow_gyroy,gyroy_imufilter,filter_gyroy);
   static double f1_b;//f1_b 是融合比例。
    if(valid==245)		  
	{
       if(f1_b<1.2)
			{
				f1_b += 0.02;
			}	
	}
	else
	{
		f1_b = 0.5;
	}
  f1_fx.out=imu_Rate.vx;//cm/s
  f1_fy.out=imu_Rate.vy;//cm/s

   flow_Rate.vx=(filter_gyroy)*height;//cm/s
   flow_Rate.vy=(filter_gyrox)*height;//cm/s
    Filter_1(f1_b,2.5,dt,-flow_Rate.vx,&f1_fx);   //flow_data with acc integrated data complementary filtering
		Filter_1(f1_b,2.5,dt,flow_Rate.vy,&f1_fy);//注意光流与imuy轴方向相反，就取反，以imu的为正轴
    //加速度补偿反而漂，所以就改回来

  f1_fx.out=-flow_Rate.vx;//cm/s
  f1_fy.out=flow_Rate.vy;//cm/s
  //上面融合可以就不用
    kalman_filter(&K_Ratex, f1_fx.out);
     Temp_vx=K_Ratex.output;
    kalman_filter(&K_Ratey,f1_fy.out);
     Temp_vy=K_Ratey.output;
   //误差补偿
 Rate.vx=Temp_vx+0.1*speed_err_ix;
 Rate.vy=Temp_vy+0.1*speed_err_iy;

speed_err_ix+=(Temp_vx-flow_Rate.vx)*dt;
speed_err_iy+=(Temp_vy-flow_Rate.vy)*dt;
if(Data_clearcoor)
    {
      flow_Coor.x=flow_Coor.y=0;
       Data_clearcoor=!Data_clearcoor;
    }
   //printf("Rate:%lf,%lf\n", flow_Rate.vx,flow_Rate.vy);

}