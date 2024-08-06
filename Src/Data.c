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
#include "Kalman.h"
#include "CalculateFlow.h"
#include "BMP280.h"
#include "math.h"
#define M_PI 3.14159265358979323846
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);

static K_Filter K_height={0,0.005f,0.5f,0.012f,0,0};//需要调参

/*以下是融合处理后的数，可以直接放入pid*/
 T_Acc Acc;
  T_gyro Gyro;
  T_angle Angle;
  T_coor  Coor;//位移
  T_rate  Rate;//速度
  double height;//高度值
T_rate imu_Rate;//加速度积分的速度值
/*传感器数据*/
 double flow_height;//光流高度值

double baro_height;//气压计输出高度

 
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

 
int8_t Data_wit_Getimu(double dT)
{
    double fAcc[3], fGyro[3], fAngle[3];
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
           Acc.Ax=(double)fAcc[0]; Acc.Ay=(double)fAcc[1];Acc.Az=(double)fAcc[2];//单位g/s2
			Gyro.Gx=(double)fGyro[0];Gyro.Gy=(double)fGyro[1];Gyro.Gz=(double)fGyro[2];
			Angle.roll=(double)fAngle[0];Angle.pitch=(double)fAngle[1];Angle.yaw=(double)fAngle[2];
        //printf("imugyroandangle:%lf,%lf,%lf\n",Angle.roll,Angle.pitch,Angle.yaw);
        //printf("Acc:%lf,%lf,%lf\n",Acc.Ax,Acc.Ay,Acc.Az);

        //imu消除重力,单位m/s2
        double vec_accx,vec_accy,vec_accz;
        vec_accy=(Acc.Ay-sin(Angle.roll*M_PI / 180.0)*cos(Angle.pitch*M_PI / 180.0))*9.81;
        vec_accx=(Acc.Ax+sin(Angle.pitch*M_PI / 180.0))*9.81;
        vec_accz=(Acc.Az-cos(Angle.pitch*M_PI / 180.0)*cos(Angle.roll*M_PI / 180.0))*9.81;
      //累加速度，单位cm/s2
        imu_Rate.vx+=vec_accx*dT*10;
        imu_Rate.vy+=vec_accy*dT*10;
        //printf("imuRate:%lf,%lf\n",imu_Rate.vx,imu_Rate.vy);
			return UAVNormal;
		}

int8_t Data_upixels_flowget(double dt,double dT)
{           

		   flow_height=ground_distance;//mm
         CalculateFlow_upixels_Complementary(dt,Gyro);
        printf("Coor:%lf,%lf,%d\n", Coor.x, Coor.y,valid);
         //printf("Rate:%lf,%lf\n", flow_Rate.vx,Rate.vx);
       Coor.x=-flow_Coor.x;
      Coor.y=flow_Coor.y;
      if(Coor.x>100||Coor.x<-100)
      {
         Coor.x=0;
      }
		if(Coor.y>100||Coor.y<-100)
      {
         Coor.y=0;
      }
    
		return UAVNormal;
	
}
//输入参数：置信度;选择使用算法：1：融合；2：纯激光；3：纯气压计
int8_t Data_Height_fusion(uint8_t flag)
//输入单位均是mm，输出是cm
{
	double temp_height;
   static double Rh;
   static int8_t i=0;
  // if(i<20)
   //{
     // Rh+=BMP280_calculate_altitude()/20;
     // i++;
  // }else{
   //baro_height = 100*(BMP280_calculate_altitude()-Rh);
    //printf("Bmpheight:%f\n",baro_height);
   //}
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
double Data_limit(double data, double toplimit, double lowerlimit)
{
  if(data > toplimit)  data = toplimit;
  else if(data < lowerlimit) data = lowerlimit;
    return data;
}