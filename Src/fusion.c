/**
 * @file fusion.c
 * @author Lcoy (lcoy2004@qq.com)
 * @brief 数据融合+滤波
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "Data.h"
#include "Kalman.h"
#include "Data.h"
#include "main.h"
//R,Q,Q表示对模型的信任程度，R表示对量测的信任程度
static K_Filter K_height={0,10.0f,12.0f,0.01f,0,0};//需要调参



/*以下是融合处理后的数，可以直接放入pid*/
 T_Acc Acc;
  T_gyro Gyro;
  T_angle Angle;
  T_coor  Coor;//位移
  T_rate  Rate;//速度
  double height;//高度值

//输入参数：激光测距，气压计高度，置信度;选择使用算法：1：融合；2：纯激光；3：纯气压计
int8_t Fusion_Height(uint16_t flow_height,double baro_height,uint8_t confidence,uint8_t flag)
//输入单位均是mm，输出是cm
{
	double temp_height;
    if(flag==1)
   {
	double confi=(double)confidence/100;
    temp_height=(10.0*(double)flow_height)*confi+(10*baro_height)*(1-confi);
    kalman_filter(&K_height,temp_height);
   height=K_height.output;
   }else if(flag==2)
   {
   temp_height=(10.0*(double)flow_height);

   }else if(flag==3)
   {
	 temp_height=10*baro_height;
   }else
   {
	return UAVError;
   }
kalman_filter(&K_height,temp_height);
height=K_height.output;
return UAVNormal;
}
void Fusion_rate(T_rate flowrate,T_Acc imuacc)
{

}
