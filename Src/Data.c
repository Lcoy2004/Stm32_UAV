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
#include "main.h"
#include "Reg.h"
#include "stm32h7xx_hal.h"
#include "Data.h"
#include "usart.h"
#include "flow_decode.h"
#include "fusion.h"
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);

/*传感器数据*/
 T_Acc imu_Acc;
 T_gyro imu_Gyro;
 T_angle imu_Angle;
 T_coor  flow_Coor;//光流积分得到的值
 T_rate  flow_Rate;//光流得到的速度值
 double flow_height;//光流高度值
 static uint16_t flow1_height;//激光测距得到的光流值
 
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
           imu_Acc.Ax=(double)fAcc[0]; imu_Acc.Ay=(double)fAcc[1]; imu_Acc.Az=(double)fAcc[2];
			imu_Gyro.Gx=(double)fGyro[0];imu_Gyro.Gy=(double)fGyro[1];imu_Gyro.Gz=(double)fGyro[2];
			imu_Angle.roll=(double)fAngle[0];imu_Angle.pitch=(double)fAngle[1];imu_Angle.yaw=(double)fAngle[2];
			return UAVNormal;
		}

int8_t Data_upixels_flowget(double dt)
{           
	        double flow_gyrox,flow_gyroy;
	        int16_t flow_x_integral = 0;
	        int16_t flow_y_integral = 0;
	        uint16_t ground_distance = 0;
			uint8_t valid = 0;
		    uint8_t tof_confidence = 0;
			   flow_x_integral = up_data.flow_x_integral;
			   flow_y_integral = up_data.flow_y_integral;
			ground_distance = up_data.ground_distance;
			valid = up_data.valid;
			tof_confidence = up_data.tof_confidence;
			if(valid==245)
           {
		   flow1_height=ground_distance*10.0;//mm->cm
		   flow_gyroy=(double)flow_x_integral/10000*57.296/dt;//度/s，绕y轴的光流角速度
		   flow_gyrox=(double)flow_y_integral/10000*57.296/dt;//度/s，绕x轴的光流角速度
           flow_Coor.x+=(double)flow_x_integral/10000*(height*100);//累加位移cm
		   flow_Coor.y+=(double)flow_y_integral/10000*(height*100);//累加位移cm
           flow_Rate.vx=(double)flow_x_integral/10000*(height*100)/dt;//cm/s
		   flow_Rate.vy=(double)flow_y_integral/10000*(height*100)/dt;//cm/s
		   
			return UAVNormal;
		   }else if(valid==0)
		   {
			return UAVError;
		   }

				
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
