#include "wit_c_sdk.h"
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
//R,Q,Q表示对模型的信任程度，R表示对量测的信任程度
static K_Filter K_height={0,10.0f,12.0f,0.01f,0,0};//需要调参
/*传感器数据*/
 T_Acc Acc;
 T_gyro Gyro;
 T_angle Angle;
 T_coor  Coor;//光流积分加角度补偿得到的值
 T_rate  Rate;//光流得到的值
 double height;//高度值
 static uint16_t flow_height;//激光测距得到的光流值
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
           Acc.Ax=(double)fAcc[0]; Acc.Ay=(double)fAcc[1]; Acc.Az=(double)fAcc[2];
			Gyro.Gx=(double)fGyro[0];Gyro.Gy=(double)fGyro[1];Gyro.Gz=(double)fGyro[2];
			Angle.roll=(double)fAngle[0];Angle.pitch=(double)fAngle[1];Angle.yaw=(double)fAngle[2];
			return UAVNormal;
		}
int8_t Data_upixels_flowget(double dt)
{
	        int16_t flow_x_integral = 0;
	        int16_t flow_y_integral = 0;
	        uint16_t ground_distance = 0;
			uint8_t valid = 0;
		    uint8_t tof_confidence = 0;
			   flow_x_integral = up_data.flow_x_integral;
			   flow_y_integral = up_data.flow_y_integral;
			  flow_height = up_data.ground_distance;
			valid = up_data.valid;
			tof_confidence = up_data.tof_confidence;
           height=ground_distance*10.0;//mm->cm

           
			return UAVNormal;
				
}

//输入参数：激光测距，气压计高度，置信度，选择使用算法：1：融合；2：纯激光；3：纯气压计
int8_t Data_Height_fusion(uint16_t flow_height,double baro_height,uint8_t confidence,uint8_t flag)//输入单位均是mm
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
