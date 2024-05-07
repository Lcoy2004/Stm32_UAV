#include "mymath.h"
#include "imu.h"
#include <math.h>
//绕X轴旋转角度为roll，绕Y轴旋转角度为pitch，绕Z轴旋转角度为yaw
//对于角速度进行PID控制
void imu_Getangle(MPU6050_gyro gyro,MPU6050_Acc Acc,T_angle *angle,float dt)
{
	volatile struct V
	{
		float x;
		float y;
		float z;
	} Gravity,Acct,Gyrot,AccGravity;

	static struct V GyroIntegError = {0};
	static  float KpDef = 0.8f ;// 角速度融合加速度比例补偿值系数
	static  float KiDef = 0.0003f;
	static Quaternion NumQ = {1, 0, 0, 0};
	float q0_t,q1_t,q2_t,q3_t;
    float NormAcc;	
	float NormQuat; 
	float HalfTime = dt * 0.5f;

	// 提取等效旋转矩阵中的重力分量 
	Gravity.x = 2*(NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);								
	Gravity.y = 2*(NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);						  
	Gravity.z = 1-2*(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);	
	// 加速度归一化
	NormAcc = Q_rsqrt(squa(Acc.Ax)+ squa(Acc.Ay) +squa(Acc.Az));

	Acct.x = Acc.Ax * NormAcc;
	Acct.y = Acc.Ay * NormAcc;
	Acct.z = Acc.Az * NormAcc;	
	//向量差乘得出的值
	AccGravity.x = (Acct.y * Gravity.z - Acct.z * Gravity.y);
	AccGravity.y = (Acct.z * Gravity.x - Acct.x * Gravity.z);
	AccGravity.z = (Acct.x * Gravity.y - Acct.y * Gravity.x);
	//再做加速度积分补偿角速度的补偿值，求姿态误差
	GyroIntegError.x += AccGravity.x * KiDef;
	GyroIntegError.y += AccGravity.y * KiDef;
	GyroIntegError.z += AccGravity.z * KiDef;
	//角速度融合加速度积分补偿值
	Gyrot.x = gyro.Gx * Gyro_Gr + KpDef * AccGravity.x  +  GyroIntegError.x;//弧度制
	Gyrot.y = gyro.Gy * Gyro_Gr + KpDef * AccGravity.y  +  GyroIntegError.y;
	Gyrot.z = gyro.Gz * Gyro_Gr + KpDef * AccGravity.z  +  GyroIntegError.z;		
	// 一阶龙格库塔法, 更新四元数

	q0_t = (-NumQ.q1*Gyrot.x - NumQ.q2*Gyrot.y - NumQ.q3*Gyrot.z) * HalfTime;
	q1_t = ( NumQ.q0*Gyrot.x - NumQ.q3*Gyrot.y + NumQ.q2*Gyrot.z) * HalfTime;
	q2_t = ( NumQ.q3*Gyrot.x + NumQ.q0*Gyrot.y - NumQ.q1*Gyrot.z) * HalfTime;
	q3_t = (-NumQ.q2*Gyrot.x + NumQ.q1*Gyrot.y + NumQ.q0*Gyrot.z) * HalfTime;

	NumQ.q0 += q0_t;
	NumQ.q1 += q1_t;
	NumQ.q2 += q2_t;
	NumQ.q3 += q3_t;
	// 四元数归一化
	NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
	NumQ.q0 *= NormQuat;
	NumQ.q1 *= NormQuat;
	NumQ.q2 *= NormQuat;
	NumQ.q3 *= NormQuat;	
// 四元数转欧拉角
	{ 
		#ifdef	YAW_GYRO
		angle->yaw = atan2f(2 * NumQ.q1 *NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 *NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA;   //yaw
		#else
			float yaw_G = gyro.Gz * Gyro_G;
			if((yaw_G > 5.0f) || (yaw_G < -5.0f)) //不让yaw静止时漂移
			{
				angle->yaw  += yaw_G * dt;			
			}
		#endif
		angle->pitch  =  asin(2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3) * RtA;						
		angle->roll	= atan2(2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1, 1 - 2 * NumQ.q1 *NumQ.q1 - 2 * NumQ.q2 * NumQ.q2) * RtA;	//PITCH 			
	}			
	}