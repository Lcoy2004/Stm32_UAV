/**
 * @file Data.c
 * @author Lcoy (lcoy2004@qq.com)
 * @brief 获取数据及简单处理
 * @version 1.0
 * @date 2024-05-27
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "wit_c_sdk.h"
#include "stdio.h"
#include "main.h"
#include "Reg.h"
#include "stm32h7xx_hal.h"
#include "Data.h"
#include "usart.h"
#include "flow_decode.h"
#include "Kalman.h"
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *3.14f *(t) ) ) ) *( (in) - (out) ))	//一阶低通滤波
/*hz：滤波器的截止频率（Hz）。
t：时间步长（秒）。这通常是两次连续采样之间的时间差。
in：输入值，即当前时刻的采样值。
out：输出值，即经过滤波器处理后的值。*/
//R,Q,Q表示对模型的信任程度，R表示对量测的信任程度
static K_Filter K_height={0,0.05f,0.5f,0.012f,0,0};//需要调参
static K_Filter K_Ratex={0,0.52f,12.0f,0.01f,0,0};//需要调参
static K_Filter K_Ratey={0,0.52f,12.0f,0.01f,0,0};//需要调参
double data_limit(double data, double toplimit, double lowerlimit);


/*以下是融合处理后的数，可以直接放入pid*/
 T_Acc Acc;
  T_gyro Gyro;
  T_angle Angle;
  T_coor  Coor;//位移
  T_rate  Rate;//速度
  double height;//高度值

/*传感器数据*/
 T_coor  flow_Coor;//光流积分得到的值
 T_rate  flow_Rate;//光流得到的速度值
 double flow_height;//光流高度值

  double baro_height;//气压计输出高度

 int16_t flow_x_integral = 0;
 int16_t flow_y_integral = 0;
 uint16_t ground_distance = 0;
 uint8_t valid = 0;
 uint8_t tof_confidence = 0;
  /**
   * @brief witmiu数据获取初始化
   * 
   * @return int8_t 
   */
int8_t Data_wit_Init()
{
WitInit(WIT_PROTOCOL_NORMAL, 0x50);
WitDelayMsRegister(Delayms);	
WitSerialWriteRegister(SensorUartSend);
WitRegisterCallBack(SensorDataUpdata);
WitSetUartBaud(WIT_BAUD_115200);
WitSetOutputRate(RRATE_200HZ);	
return UAVNormal;
}

 
int8_t Data_wit_Getimu()
{
    float fAcc[3], fGyro[3], fAngle[3];
     int8_t i=0;
		if(s_cDataUpdate)
		{
			for(i = 0; i < 3; i++)
			{
				fAcc[i]= sReg[AX+i] / 32768.0f * 16.0f;
				fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
				fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			}
			if(s_cDataUpdate & ACC_UPDATE)
				s_cDataUpdate &= ~ACC_UPDATE;
			}
			if(s_cDataUpdate & GYRO_UPDATE)
			{
				s_cDataUpdate &= ~GYRO_UPDATE;
			}
			if(s_cDataUpdate & ANGLE_UPDATE)
			{
				s_cDataUpdate &= ~ANGLE_UPDATE;
			}
			if(s_cDataUpdate & MAG_UPDATE)
			{
				s_cDataUpdate &= ~MAG_UPDATE;
			}
           Acc.Ax=(double)fAcc[0]; Acc.Ay=(double)fAcc[1];Acc.Az=(double)fAcc[2];
			Gyro.Gx=(double)fGyro[0];Gyro.Gy=(double)fGyro[1];Gyro.Gz=(double)fGyro[2];
			Angle.roll=(double)fAngle[0];Angle.pitch=(double)fAngle[1];Angle.yaw=(double)fAngle[2];
			return UAVNormal;
		}
//dt:光流采样；dT：imu采样
//优象光流的xy轴与imu不太一致。若对齐x轴，两个y轴相反，因此这里默认imuy轴为飞机y轴，光流的y轴取反
int8_t Data_upixels_flowget(double dt,double dT)
{           
			double flow_gyrox,flow_gyroy;
		   flow_gyroy=(double)flow_x_integral/10000/dt;//rad/s，绕y轴的光流角速度
		   flow_gyrox=(double)flow_y_integral/10000/dt;//rad/s，绕x轴的光流角速度
		   flow_height=ground_distance;//mm
           flow_Coor.x+=((double)flow_x_integral/10000)*(height);//累加位移cm
		   flow_Coor.y+=(double)flow_y_integral/10000*(height);//累加位移cm
           flow_Rate.vx=(double)flow_x_integral/10000*(height)/dt;//cm/s
		   flow_Rate.vy=(double)flow_y_integral/10000*(height)/dt;//cm/s
      double gyrox_imufilter,gyroy_imufilter,filter_gyrox,filter_gyroy;
          LPF_1_(2.0,dT,Gyro.Gx,gyrox_imufilter);     //gyro low pass filter (delay) for fitting flow data()
        LPF_1_(2.0,dT,Gyro.Gy,gyroy_imufilter);
      //printf("imuandflow:%lf,%lf\n",gyrox_imufilter/2,flow_gyrox);//测定相位是否一致
   filter_gyrox=(flow_gyrox-(data_limit(gyrox_imufilter,1.0,-1.0)/57.3))/dt;
   filter_gyroy=(flow_gyroy-(data_limit(gyroy_imufilter,1.0,-1.0)/57.3))/dt;
   Rate.vx=-(filter_gyroy)*height;//cm/s
   Rate.vy=(filter_gyrox)*height;//cm/s
   printf("flowrate:%lf,%lf\n",flow_Rate.vx,Rate.vx);
           kalman_filter(&K_Ratex,Rate.vx);
          Rate.vx=K_Ratex.output;
         kalman_filter(&K_Ratey,Rate.vy);
          Rate.vy=K_Ratey.output;
          Coor.x+=Rate.vx*dt;
          Coor.y+=Rate.vy*dt;	
    if(valid==245)		  
	{
		return UAVNormal;
	}
	else
	{
		return UAVError;
	}
}
//输入参数：置信度;选择使用算法：1：融合；2：纯激光；3：纯气压计
int8_t Data_Height_fusion(uint8_t flag)
//输入单位均是mm，输出是cm
{
	double temp_height;
    if(flag==1)
   {
	double confi=(double)tof_confidence/100;
    temp_height=((double)flow_height/10)*confi+(baro_height/10)*(1-confi);
   height=K_height.output;
   }else if(flag==2)
   {
   temp_height=(double)flow_height/10;

   }else if(flag==3)
   {
	 temp_height=baro_height/10;
   }else
   {
	return UAVError;
   }

kalman_filter(&K_height,temp_height);
height=K_height.output;
//printf("heightkal:%f,%f\n",temp_height,height);
return UAVNormal;
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
    HAL_UART_Transmit_DMA(&huart1,p_data,uiSize);
}

static void Delayms(uint16_t ucMs)
{
	 HAL_Delay(ucMs);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}
double data_limit(double data, double toplimit, double lowerlimit)
{
  if(data > toplimit)  data = toplimit;
  else if(data < lowerlimit) data = lowerlimit;
    return data;
}