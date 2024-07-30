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
double *Num;
int8_t w;//状态
uint8_t UAV_Flymode=1;//uav模式选择，0：自主悬停，1：遥控控制（3.测试模式）

/**
 * @brief 获取期望目标
 * 
 * @return void 
 */
void Remote_Updata(int8_t ch)
{
  static int qe;
  static int8_t qw;//判断正负号
switch (w)
{
 //包头三个数据mac地址与信道，不处理
case 0:
qw=Remote_flag(ch);//标志位判断函数（包括-1判断）
//printf("%02X\n",ch);
  break;
case 1:/* kix */ 
       (*Num)+=(1000* ch * qw); 
       w++;
  break;
  case 2:/*kid */
    (*Num)+=(10*ch*qw);
    target_angle.pitch=Data_limit(target_angle.pitch,7,-7);
    target_angle.roll=Data_limit(target_angle.roll,7,-7);
    power=Data_limit(power,780,0);
    w=0;
  break;
 default:
     w=0;
  break;
}
}

int8_t Remote_flag(uint8_t ch)
{
  static uint8_t k;
  
  Remote_connectcheck=1;
switch (ch)
{
case 0x73:w++;//空指令
if(k==0)
{
Remote_hover_flag=1;
Coor.x=0;Coor.y=0;
t_height=height;
k++;
}
Num=NULL;
target_angle.pitch=0;target_angle.roll=-2;

break;
case 0x6D:w++;//左旋
Num = &target_angle.yaw;
Remote_hover_flag=0;
k=0;//等待下次遥杆回正记录预期高度
return 1;//正数
break;
case 0x6E:w++;//右旋
Num = &target_angle.yaw;
Remote_hover_flag=0;
k=0;//等待下次遥杆回正记录预期高度
return -1;//负数
break;
case 0x6C :w++;//向左
Num = &target_angle.pitch;
Remote_hover_flag=0;
k=0;//等待下次遥杆回正记录预期高度
return 1;
break;
case 0x72 :w++;//向右
Num = &target_angle.pitch;
Remote_hover_flag=0;
k=0;//等待下次遥杆回正记录预期高度
return -1;
break;
case 0x66 :w++;//向前
Remote_hover_flag=0;
Num = &target_angle.roll;
k=0;//等待下次遥杆回正记录预期高度
return 1;
break;
case 0x62 :w++;//向后
Remote_hover_flag=0;
Num = &target_angle.roll;
k=0;//等待下次遥杆回正记录预期高度
return -1;
break;
case 0x75 :w++;//高度
Num = &power;
Remote_hover_flag=0;
k=0;//等待下次遥杆回正记录预期高度
return 1;
case 0x64 :w++;//高度
Num = &power;
Remote_hover_flag=0;
k=0;//等待下次遥杆回正记录预期高度
return -1;
break;
case 0x13 :w++;//急停
Num = NULL;
k=0;
UAV_stop_flag=1;
break;
case 0x14 :w++;//模式切换
UAV_Flymode=!UAV_Flymode;
Num = NULL;
break;
case 0xC9 :w++;
 PID_coordx.kp+=0.01;
Num = NULL;
break;
case 0xCA :w++;
 PID_coordx.ki+=0.01;
Num = NULL;
break;
case 0xCB :w++;
 PID_coordx.kd+=0.01;
Num = NULL;
break;
case 0xCC :w++;
PID_ratex.kp+=0.01;
Num = NULL;
break;
case 0xCD :w++;
PID_ratex.ki+=0.01;
Num = NULL;
break;
case 0xCE :w++;
PID_ratex.kd+=0.01;
Num = NULL;
break;
case 0xCF :w++;
PID_coordy.kp+=0.01;
Num = NULL;
break;
case 0xD0 :w++;
PID_coordy.ki+=0.01;
Num = NULL;
break;
case 0xD1 :w++;
PID_coordy.kd+=0.01;
Num = NULL;
break;
case 0xD2 :w++;
PID_ratey.kp+=0.01;
Num = NULL;
break;
case 0xD3 :w++;
PID_ratey.ki+=0.01;
Num = NULL;
break;
case 0xD4 :w++;
PID_ratey.kd+=0.01;
Num = NULL;
break;
case 0xD5 :w++;
PID_height.kp+=0.01;
Num = NULL;
break;
case 0xD6 :w++;
PID_height.ki+=0.01;
Num = NULL;
break;
case 0xD7 :w++;
PID_height.kd+=0.01;
Num = NULL;
break;

case 0xDB :w++;
 PID_coordx.kp-=0.01;
Num = NULL;
break;
case 0xDC :w++;
 PID_coordx.ki-=0.01;
Num = NULL;
break;
case 0xDD :w++;
 PID_coordx.kd-=0.01;
Num = NULL;
break;
case 0xDE :w++;
 PID_coordx.kp-=0.01;
Num = NULL;
break;
case 0xDF :w++;
PID_ratex.ki-=0.01;
Num = NULL;
break;
case 0xE0 :w++;
PID_ratex.kd-=0.01;
Num = NULL;
break;
case 0xE1 :w++;
PID_coordy.kp-=0.01;
Num = NULL;
break;
case 0xE2 :w++;
PID_coordy.ki-=0.01;
Num = NULL;
break;
case 0xE3 :w++;
PID_coordy.kd-=0.01;
Num = NULL;
break;
case 0xE4 :w++;
PID_ratey.kp-=0.01;
Num = NULL;
break;
case 0xE5 :w++;
PID_ratey.ki-=0.01;
Num = NULL;
break;
case 0xE6 :w++;
PID_ratey.kd-=0.01;
Num = NULL;
break;
case 0xE7 :w++;
PID_height.kp-=0.01;
Num = NULL;
break;
case 0xE8 :w++;
PID_height.ki-=0.01;
Num = NULL;
break;
case 0xE9 :w++;
PID_height.kd-=0.01;
Num = NULL;
break;


case 0x15 :w++;
PID_roll.kp+=0.01;
Num = NULL;
break;
case 0x16 :w++;
PID_roll.ki+=0.01;
Num = NULL;
break;
case 0x17 :w++;
PID_roll.kd+=0.01;
Num = NULL;
case 0x18 :w++;
PID_gyrox.kp+=0.01;
Num = NULL;
break;
case 0x19 :w++;
PID_gyrox.ki+=0.01;
Num = NULL;
break;
case 0x1A :w++;
PID_gyrox.kd+=0.01;
Num = NULL;
break;
case 0x1B :w++;
PID_pitch.kp+=0.01;
Num = NULL;
break;
case 0x1C :w++;
PID_pitch.ki+=0.01;
Num = NULL;
break;
case 0x1D :w++;
PID_pitch.kd+=0.01;
Num = NULL;
case 0x1E :w++;
 PID_gyroy.kp+=0.01;
Num = NULL;
break;
case 0x1F :w++;
 PID_gyroy.ki+=0.01;
Num = NULL;
break;
case 0x20 :w++;
 PID_gyroy.kd+=0.01;
Num = NULL;
break;
case 0x21 :w++;
PID_yaw.kp+=0.001;
Num = NULL;
break;
case 0x22 :w++;
PID_yaw.ki+=0.001;
Num = NULL;
break;
case 0x23 :w++;
PID_yaw.kd+=0.001;
Num = NULL;
case 0x24 :w++;
PID_gyroz.kp+=0.001;
Num = NULL;
break;
case 0x25 :w++;
PID_gyroz.ki+=0.001;
Num = NULL;
break;
case 0x26 :w++;
PID_gyroz.kd+=0.001;
Num = NULL;
break;



case 0x27 :w++;
PID_roll.kp-=0.01;
Num = NULL;
break;
case 0x28 :w++;
PID_roll.ki-=0.01;
Num = NULL;
break;
case 0x29 :w++;
PID_roll.kd-=0.01;
Num = NULL;
case 0x2A :w++;
PID_gyrox.kp-=0.01;
Num = NULL;
break;
case 0x2B :w++;
PID_gyrox.ki-=0.01;
Num = NULL;
break;
case 0x2C :w++;
PID_gyrox.kd-=0.01;
Num = NULL;
break;
case 0x2D :w++;
PID_pitch.kp-=0.01;
Num = NULL;
break;
case 0x2E :w++;
PID_pitch.ki-=0.01;
Num = NULL;
break;
case 0x2F :w++;
PID_pitch.kd-=0.01;
Num = NULL;
case 0x30 :w++;
 PID_gyroy.kp-=0.01;
Num = NULL;
break;
case 0x31 :w++;
 PID_gyroy.ki-=0.01;
Num = NULL;
break;
case 0x32 :w++;
 PID_gyroy.kd-=0.01;
Num = NULL;
break;
case 0x33 :w++;
PID_yaw.kp-=0.001;
Num = NULL;
break;
case 0x34 :w++;
PID_yaw.ki-=0.001;
Num = NULL;
break;
case 0x35 :w++;
PID_yaw.kd-=0.001;
Num = NULL;
case 0x36 :w++;
 PID_gyroz.kp-=0.001;
Num = NULL;
break;
case 0x37 :w++;
 PID_gyroz.ki-=0.001;
Num = NULL;
break;
case 0x38 :w++;
 PID_gyroz.kd-=0.001;
Num = NULL;
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