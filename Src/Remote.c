#include "Remote.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "Data.h"
#include "usart.h"
#include "stdio.h"
#include "Control.h"
#include "State.h"
uint8_t   Remote_hover_flag;//完成动作后将预期高度设置成目前高度
T_angle target_angle;//获取的目标角度
double t_height;//送入pid的期望高度
double power;//马力
double t_coodx;
double t_coody;//预期距离y
double openmv_coody;//openmv发送的数据
uint8_t Remote_connectcheck;//0表示断连
uint8_t PID_Flag;
double *Num;
int8_t w;//状态
/**
 * @brief 从遥控端获得期望模式：悬停/遥控控制
 * 
 * @return int8_t 
 */
uint8_t UAV_Flymode=1;//uav模式选择，0：自主悬停，1：遥控控制（3.测试模式）

/**
 * @brief 获取期望目标
 * 
 * @return void 
 */
void Remote_Updata(int8_t ch)
{
  static int qe;
  static uint8_t pn;
  static int8_t qw;//判断正负号
switch (w)
{
 //包头三个数据，不处理
case 0:
qw=Remote_flag(ch);//标志位判断函数（包括-1判断）
//printf("%02X\n",ch);//厉害
  break;
case 1:/* kix */ 
 if(PID_Flag)
 {
  pn=( ch * qw);
 }
 else
      {
       (*Num)+=(0.1* ch * qw); 
      } 
       w++;
  break;
  case 2:/*kid */
     if(PID_Flag)
    {
      (*Num)=(0.01* ch * qw)+pn;
    }
 else
      {
    (*Num)+=(0.001*ch*qw);
    target_angle.pitch=Data_limit(target_angle.pitch,15,-15);
    target_angle.roll=Data_limit(target_angle.roll,15,-15);
    power=Data_limit(power,750,0);
      }
    w=0;
  break;
 default:
     w=0;
  break;
}
}

int8_t Remote_flag(int8_t ch)
{
  static uint8_t k;
  Remote_hover_flag=0;
  Remote_connectcheck=1;
  PID_Flag=0;
switch (ch)
{
case -1:Remote_connectcheck=0;
  /* code */
  break;
case 0x73:w++;//空指令
if(k==0)
{
Remote_hover_flag=1;
k++;
}
Num=NULL;
target_angle.pitch=0;target_angle.roll=0;
break;
case 0x6D:w++;//左旋
Num = &target_angle.yaw;
k=0;//等待下次遥杆回正记录预期高度
return 1;//正数
break;
case 0x6E:w++;//右旋
Num = &target_angle.yaw;
k=0;//等待下次遥杆回正记录预期高度
return -1;//负数
break;
case 0x6C :w++;//向左
Num = &target_angle.pitch;
k=0;//等待下次遥杆回正记录预期高度
return -1;
break;
case 0x72 :w++;//向右
Num = &target_angle.pitch;
k=0;//等待下次遥杆回正记录预期高度
return 1;
break;
case 0x66 :w++;//向前
Num = &target_angle.roll;
k=0;//等待下次遥杆回正记录预期高度
return 1;
break;
case 0x62 :w++;//向后
Num = &target_angle.roll;
k=0;//等待下次遥杆回正记录预期高度
return -1;
break;
case 0x75 :w++;//高度
Num = &power;
k=0;//等待下次遥杆回正记录预期高度
return 1;
case 0x64 :w++;//高度
Num = &power;
k=0;//等待下次遥杆回正记录预期高度
return -1;
break;
case 0x13 :w++;//急停
Num = NULL;
UAV_stop_flag=1;
break;
case 0x14 :w++;//模式切换
UAV_Flymode=!UAV_Flymode;
Num = NULL;
break;
case 0x01 :w++;
Num = &PID_roll.kp;
PID_Flag=1;
break;
case 0x02 :w++;
Num = &PID_roll.ki;
PID_Flag=1;
break;
case 0x03 :w++;
Num = &PID_roll.kd;
PID_Flag=1;
break;
case 0x04 :w++;
Num = &PID_gyrox.kp;
PID_Flag=1;
break;
case 0x05 :w++;
Num = &PID_gyrox.ki;
PID_Flag=1;
break;
case 0x06 :w++;
Num = &PID_gyrox.kd;
PID_Flag=1;
break;
case 0x07 :w++;
Num = &PID_pitch.kp;
PID_Flag=1;
break;
case 0x08 :w++;
Num = &PID_pitch.ki;
PID_Flag=1;
break;
case 0x09 :w++;
Num = &PID_pitch.kd;
PID_Flag=1;
break;
case 0x0A :w++;
Num = &PID_gyroy.kp;
PID_Flag=1;
break;
case 0x0B :w++;
Num = &PID_gyroy.ki;
PID_Flag=1;
break;
case 0x0C :w++;
Num = &PID_gyroy.kd;
PID_Flag=1;
break;
case 0x0D :w++;
Num = &PID_yaw.kp;
PID_Flag=1;
case 0x0E :w++;
Num = &PID_yaw.ki;
PID_Flag=1;
break;
case 0x0F :w++;
Num = &PID_yaw.kd;
PID_Flag=1;
break;
case 0x10 :w++;
Num = &PID_gyroz.kp;
PID_Flag=1;
break;
case 0x11 :w++;
Num = &PID_gyroz.ki;
PID_Flag=1;
break;
case 0x12 :w++;
Num = &PID_gyroz.kd;
PID_Flag=1;
break;
default:
w=0;
break;
}
return 1;
}
uint8_t Remote_openmv_Flag(uint8_t ch,uint8_t *q)
{
switch (ch)
{
case 0x61:
(*q)++;
  return 0;
  break;
case 0x62:
  (*q)++;
  return 1;
  break;
default:
(*q)=0;
  break;
}
return 1;
}
void Remote_openmv_Updata(uint8_t ch)
{
  static uint8_t q;
  uint8_t qe;
  printf("t_coody:%lf\n",t_coody);
  switch (q)
{
  case 0:qe=Remote_openmv_Flag(ch,&q);
    break;
   case 1:if(qe&&!UAV_Flymode)
   {
    target_angle.yaw=(double)ch;
    q++;
   }else if(!qe&&!UAV_Flymode)
   {
    openmv_coody=(double)ch;
  
    q++;
   }else
   {
    q=0;
   };
    case 3:if(ch==0x0A){}else{q=0;}
    break;
  default:
  q=0;
    break;
  }
}