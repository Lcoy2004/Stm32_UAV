#include "filter.h"
#include "Data.h"
#include "stdio.h"
#include "main.h"
#include "CalculateFlow.h"
int16_t flow_x_integral = 0;
 int16_t flow_y_integral = 0;
 uint16_t ground_distance ;
uint8_t valid ;
 uint8_t tof_confidence ;

/*传感器数据*/
 T_coor  flow_Coor;//光流积分得到的值
 T_rate  flow_Rate;//光流得到的速度值

 //dt:光流采样；dT：imu采样
//优象光流的xy轴与imu不太一致。若对齐x轴，两个y轴相反，因此这里默认imuy轴为飞机y轴，光流的y轴取反
void CalculateFlow_upixels_Complementary(double dt,double dT,T_gyro gyro)
{
double flow_gyrox,flow_gyroy;
//T_rate com_rate融合补偿后的速度
		   flow_gyroy=(double)flow_x_integral/10000/dt;//rad/s，绕y轴的光流角速度
		   flow_gyrox=(double)flow_y_integral/10000/dt;//rad/s，绕x轴的光流角速度
           flow_Coor.x+=((double)flow_x_integral/10000)*(height);//累加位移cm
		   flow_Coor.y+=(double)flow_y_integral/10000*(height);//累加位移cm
          // flow_Rate.vx=(double)flow_x_integral/10000*(height)/dt;//cm/s
		   //flow_Rate.vy=(double)flow_y_integral/10000*(height)/dt;//cm/s
    double gyrox_imufilter,gyroy_imufilter,filter_gyrox,filter_gyroy;
         //进行低通滤波
       // printf("imuandflow:%lf,%lf\n",Gyro.Gx,flow_gyrox);//测定相位是否一致
     Filter_low_pass_filter(gyro.Gx, &gyrox_imufilter, 3.0, dT);
     Filter_low_pass_filter(gyro.Gy, &gyroy_imufilter, 3.0, dT);
     //printf("imuandflow:%lf,%lf\n",gyrox_imufilter,flow_gyrox);//测定相位是否一致
     //printf("imuandflow:%lf,%lf\n",gyroy_imufilter,flow_gyroy);//测定相位是否一致
      //融合补偿
     filter_gyrox=flow_gyrox+18*Filter_limit((gyrox_imufilter/57.3),1.0,-1.0);//低通滤波后gyro有衰减，所以乘以固定值18
     filter_gyroy=flow_gyroy+18*Filter_limit((gyroy_imufilter/57.3),1.0,-1.0);
   //printf("flowgyro:%lf,%lf,%lf\n",flow_gyrox,gyrox_imufilter,filter_gyrox);
   //printf("flowgyro:%lf,%lf,%lf\n",flow_gyroy,gyroy_imufilter,filter_gyroy);
   flow_Rate.vx=(filter_gyroy)*height;//cm/s
   flow_Rate.vy=-(filter_gyrox)*height;//cm/s
   //printf("Rate:%lf,%lf\n", flow_Rate.vx,flow_Rate.vy);

}