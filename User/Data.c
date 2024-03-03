#include "BMP280.h"
#include "MPU6050.h"
#include "imu.h"
#include "Kalman.h"
#include "Serial.h"
#include "myMath.h"
#include "Delay.h"
#define MPU6050_ACCimu      0.0048f//换算加速度，单位m/s2
 T_Acc P_Acc;//存储加速度值测量值
 T_gyro gyro;//存储滤波后角速度值
 T_angle angle;//存储滤波后姿态角值
 float height;//存储滤波后高度值
 float P_height;//存储测量高度值
 T_gyro P_gyro;//存储角速度测量值
 T_angle P_angle;//存储角度测量值
//R,Q,Q表示对模型的信任程度，R表示对量测的信任程度
K_Filter K_P_angle={0,0.05f,0.2f,0.1f,0,0};//pitch需要调参
 K_Filter K_Y_angle={0,0.05f,0.25f,0.1f,0,0};//yaw需要调参
 K_Filter K_R_angle={0,0.05f,0.2f,0.08f,0,0};//roll需要调参
K_Filter K_height={0,0.1f,0.25f,0.07f,0,0};//需要调参
//消除零偏误差
/*int16_t R_Gx,R_Gy,R_Gz,R_Ax,R_Ay,R_Az;//零偏误差，温度不同也会变化
float R_h;//得到当前海拔高度

void Data_start()
{
int16_t Ax[25],Ay[25],Az[25],Gx[25],Gy[25],Gz[25];
int8_t i=0;
int16_t ax,ay,az,gx,gy,gz;
float h;
for(i=0;i<25;i++)
{
MPU6050_GetData(&Ax[i],&Ay[i],&Az[i],&Gx[i],&Gy[i],&Gz[i]);
h+=BMP280_calculate_altitude();
ax+=Ax[i];
ay+=Ay[i];
az+=Az[i];
gx+=Gx[i];
gy+=Gy[i];
gz+=Gz[i];
Delay_ms(5);
} 
R_Gx=(int16_t)(gx/25);R_Gy=(int16_t)(gy/25);R_Gz=(int16_t)(gz/25);
R_Ax=(int16_t)(ax/25);R_Ay=(int16_t)(ay/25);R_Az=(int16_t)(az/25);
R_h=h/25.0f;
}

void Data_Calibrate(MPU6050_Acc *MA,MPU6050_gyro *MG,float *height)
{
MA->Ax-=R_Ax;
MA->Ay-=R_Ay;
MA->Az-=R_Az;
MG->Gx-=R_Gx;
MG->Gy-=R_Gy;
MG->Gz-=R_Gz;
*height-=R_h;
}*/
 void Acc_to_imu(int16_t Ax,int16_t Ay,int16_t Az)//加速度换算
 {
  P_Acc.Ax=(float)Ax*MPU6050_ACCimu;
  P_Acc.Ay=(float)Ay*MPU6050_ACCimu;
  P_Acc.Az=(float)Az*MPU6050_ACCimu;
 }
 void Gyro_to_imu2(int16_t Gx,int16_t Gy,int16_t Gz)//换算角度
 {
  P_gyro.Gx=Gx*Gyro_G;
 P_gyro.Gy=Gy*Gyro_G;
 P_gyro.Gz=Gz*Gyro_G;
 }
 
//一阶互补滤波
float firstOrderFilter(float newValue, float oldValue, float a)
{
	return a * newValue + (1-a) * oldValue;
}

void data_filter()
{
 kalman_filter(&K_P_angle,P_angle.pitch);
angle.pitch = K_P_angle.output;

kalman_filter(&K_R_angle,P_angle.roll);
angle.roll = K_R_angle.output;

kalman_filter(&K_Y_angle,P_angle.yaw);
angle.yaw = K_Y_angle.output;

//角速度进行一阶互补滤波
gyro.Gx=firstOrderFilter(P_gyro.Gx, gyro.Gx, 0.2);

gyro.Gy=firstOrderFilter(P_gyro.Gy, gyro.Gy, 0.2);

gyro.Gz=firstOrderFilter(P_gyro.Gz, gyro.Gz, 0.2);

//高度kalman滤波
kalman_filter(&K_height,P_height);
height=K_height.output;
}

void Data_pitch_SerialTest()
{
Serial_Printf("{Kpitch:%.2f %.2f \n}",P_angle.pitch,angle.pitch);
}

void Data_row_SerialTest()
{
Serial_Printf("{Kroll:%.2f %.2f \n}",P_angle.roll,angle.roll);
}

void Data_yaw_SerialTest()
{
Serial_Printf("{Kyaw: %.2f  %.2f \n}",P_angle.yaw,angle.yaw);
}

void Data_Gyrox_SerialTest()
{
Serial_Printf("{Kgyrox: %.2f  %.2f \n}",P_gyro.Gx,gyro.Gx);
}

void Data_Gyroy_SerialTest()
{
Serial_Printf("{Kgyroy: %.2f  %.2f \n}",P_gyro.Gy,gyro.Gy);
}

void Data_Gyroz_SerialTest()
{
Serial_Printf("{Kgyroz: %.2f  %.2f \n}",P_gyro.Gz,gyro.Gz);
}
void Data_height_SerialTest()
{
Serial_Printf(" %.2f , %.2f\n",P_height,height);

}
void Data_angle_SerialTest()
{
Serial_Printf(" %f , %f,%f \n",angle.pitch,angle.roll,angle.yaw);

}
void Data_t_height_SerialTest()
{
extern float t_height;
Serial_Printf(" %.2f \n",t_height);
}
void Data_t_pitch_SerialTest()
{
extern float t_pitch;
Serial_Printf(" %.2f \n",t_pitch);
}
void Data_t_roll_SerialTest()
{
extern float t_roll;
Serial_Printf(" %.2f \n",t_roll);
}
void Data_t_yaw_SerialTest()
{
extern float t_yaw;
Serial_Printf(" %.2f \n",t_yaw);
}