#include "Remote.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "Data.h"
#include "usart.h"
T_angle target_angle;//获取的目标角度
double target_height;//获取的期望高度
double t_height;//送入pid的期望高度
double t_coodx;
double t_coody;
uint8_t Remote_connectcheck;//0表示断连
double *Num;
/**
 * @brief 从遥控端获得期望模式：悬停/遥控控制
 * 
 * @return int8_t 
 */
uint8_t UAV_Flymode=1;//uav模式选择，0：自主悬停，1：遥控控制（3.测试模式）

/**
 * @brief 获取期望目标
 * 
 * @return int8_t 
 */
void Remote_Updata(int8_t ch)
{
  static uint8_t w;
  uint8_t qw;//判断正负号
switch (w)
{
case 0:qw=Remote_flag(ch);//标志位判断函数（包括-1判断）
   w++;
  break;
case 1:/* kix */
   if( Remote_connectcheck==1)
    {
    (*Num)=100*ch*qw;
    w++;
    }
  break;
  case 2:/*kid */
  if(Remote_connectcheck==1)
    {
     (*Num)=ch*qw;
       w=0;
    }
  break;
 default:
     w=0;
  break;
}
}

int8_t Remote_flag(int8_t ch)
{
  Remote_connectcheck=1;
switch (ch)
{
case -1:Remote_connectcheck=0;
  /* code */
  break;
  case 0x73://空指令
    return 1;
   case 0x00:UAV_Flymode=!UAV_Flymode;//模式切换
return 1;
break;
case 0x6d://左旋
Num=&target_angle.yaw;
return 1;//正数
break;
case 0x6e ://右旋
Num=&target_angle.yaw;
return -1;//负数
break;
case 0x6c ://向左
Num=&target_angle.pitch;
return -1;
break;
case 0x72 ://向右
Num=&target_angle.pitch;
return 1;
break;
case 0x66 ://向前
Num=&target_angle.roll;
return 1;
break;
case 0x62 ://向后
Num=&target_angle.roll;
return -1;
break;
case 0x64 :
case 0x75 ://高度
Num=&target_height;
return 1;
break;
default:
return 1;
break;
}
}
