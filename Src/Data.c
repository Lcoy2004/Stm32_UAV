#include "wit_c_sdk.h"
#include "main.h"
#include "Reg.h"
#include "stm32h7xx_hal.h"
#include "Data.h"
#include "usart.h"

static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
/*传感器数据*/
 T_Acc Acc;
 T_gyro Gyro;
 T_angle Angle;

int8_t Data_wit_Init()
{
WitInit(WIT_PROTOCOL_NORMAL, 0x50);
WitSetUartBaud(WIT_BAUD_115200);
WitSetOutputRate(RRATE_200HZ);
WitDelayMsRegister(Delayms);	
WitSerialWriteRegister(SensorUartSend);
WitRegisterCallBack(SensorDataUpdata);
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
