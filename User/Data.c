#include "HCSR04.h"
#include "MPU6050.h"
#include "imu.h"
#include "Kalman.h"
#include "Serial.h"
#include "myMath.h"
#include "chronoscope.h"
#define MPU6050_ACCimu      0.0048f//换算加速度，单位m/s2
 T_Acc P_Acc;//存储加速度值测量值
 T_gyro gyro;//存储滤波后角速度值
 T_angle angle;//存储滤波后姿态角值
 uint16_t height;//存储滤波后高度值
 uint16_t P_height;//存储测量高度值
 T_gyro P_gyro;//存储角速度测量值
 T_angle P_angle;//存储角度测量值
float dt=0.15f;//时间间隔,R,Q,Q表示对模型的信任程度，R表示对量测的信任程度
K_Filter K_P_angle={0,0.05f,0.2f,0.1f,0,0};//pitch需要调参
 K_Filter K_Y_angle={0,0.05f,0.25f,0.1f,0,0};//yaw需要调参
 K_Filter K_R_angle={0,0.05f,0.2f,0.08f,0,0};//roll需要调参
 K_Filter K_X_Gyro={0,0.05f,0.25f,0.1f,0,0};//需要调参
K_Filter K_Y_Gyro={0,0.05f,0.2f,0.08f,0,0};//需要调参
K_Filter K_Z_Gyro={0,0.05f,0.2f,0.08f,0,0};//需要调参
K_Filter K_height={0,0.1f,0.25f,0.07f,0,0};//需要调参

int16_t R_Gx,R_Gy,R_Gz,R_Ax,R_Ay,R_Az;//零偏误差，温度不同也会变化
uint16_t R_h;
void Data_Calibrate(MPU6050_Acc *MA,MPU6050_gyro *MG,uint16_t *height)
{
MA->Ax-=R_Ax;
MA->Ay-=R_Ay;
MA->Az-=R_Az;
MG->Gx-=R_Gx;
MG->Gy-=R_Gy;
MG->Gz-=R_Gz;
*height-=R_h;
}
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
void data_filter()
{
  int16_t Ax,Ay,Az,Gx,Gy,Gz;//暂时读出MPU6050数据
  MPU6050_Acc MA;
  MPU6050_gyro MG;
//获取高度
P_height=HCSR04_GetValue();
MPU6050_GetData(&MA.Ax, &MA.Ay, &MA.Az, &MG.Gx, &MG.Gy,  &MG.Gz);
dt=Clock1_End()*0.001f;
//角度换算
Data_Calibrate(&MA,&MG,&height);//消除误差
Acc_to_imu(MA.Ax,MA.Ay,MA.Az);
Gyro_to_imu2(MG.Gx,MG.Gy,MG.Gz);
//获得姿态角
imu_Getangle(MG,MA,&P_angle,dt);//输出的角度是角度制
Clock1_Start();
//姿态角进行kalman
 kalman_filter(&K_P_angle,P_angle.pitch);
angle.pitch = K_P_angle.output;

kalman_filter(&K_R_angle,P_angle.roll);
angle.roll = K_R_angle.output;

kalman_filter(&K_Y_angle,P_angle.yaw);
angle.yaw = K_Y_angle.output;

//角速度进行kalman滤波

kalman_filter(&K_X_Gyro,P_gyro.Gx);
gyro.Gx = K_X_Gyro.output;

kalman_filter(&K_Y_Gyro,P_gyro.Gy);
gyro.Gy = K_Y_Gyro.output;

kalman_filter(&K_Z_Gyro,P_gyro.Gz);
gyro.Gz = K_Z_Gyro.output;

//高度kalman滤波
kalman_filter(&K_height,P_height);
height=K_height.output;
}
void Data_start()
{
int16_t Ax[25],Ay[25],Az[25],Gx[25],Gy[25],Gz[25];
int8_t i=0;
int16_t ax,ay,az,gx,gy,gz;
for(i=0;i<25;i++)
{
MPU6050_GetData(&Ax[i],&Ay[i],&Az[i],&Gx[i],&Gy[i],&Gz[i]);
ax+=Ax[i];
ay+=Ay[i];
az+=Az[i];
gx+=Gx[i];
gy+=Gy[i];
gz+=Gz[i];
} 
R_Gx=(int16_t)(gx/25);R_Gy=(int16_t)(gy/25);R_Gz=(int16_t)(gz/25);
R_Ax=(int16_t)(ax/25);R_Ay=(int16_t)(ay/25);R_Az=(int16_t)(az/25);
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
Serial_Printf("{Kheight: %d  %d\n}",P_height,height);

}
