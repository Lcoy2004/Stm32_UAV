#include "Remote.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "Data.h"
#include "usart.h"
#include "stdio.h"
#include "State.h"
T_angle target_angle;//获取的目标角度
double target_height;//获取的期望高度
double t_height;//送入pid的期望高度
double t_coodx;
double t_coody;
uint8_t Remote_connectcheck;//0表示断连
double *Num;
uint8_t add_flag;//赋值判断是否累加
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
  static int8_t qw;//判断正负号
switch (w)
{
case 0:qw=Remote_flag(ch);//标志位判断函数（包括-1判断）

//printf("%02X\n",ch);
  break;
case 1:/* kix */
   if(Remote_connectcheck==1)
    { 
      w++;
     
       (*Num)+=(0.01* ch * qw);  

      }
    
  break;
  case 2:/*kid */
  if(Remote_connectcheck==1)
   {
    
    (*Num)+=(0.0001*ch*qw);
    target_angle.pitch=Data_limit(target_angle.pitch,30,-30);
    target_angle.roll=Data_limit(target_angle.roll,30,-30);
    target_height=Data_limit(target_height,400,0);
    w=0;
    }
    
  break;
 default:
 add_flag=0;
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
case 0x73:w++;//空指令
Num=NULL;
target_angle.pitch=0;target_angle.roll=0;
break;
case 0x6D:w++;//左旋
Num = &target_angle.yaw;
add_flag=1;
return 1;//正数
break;
case 0x6E:w++;//右旋
Num = &target_angle.yaw;
add_flag=1;
return -1;//负数
break;
case 0x6C :w++;//向左
Num = &target_angle.pitch;
add_flag=0;
return -1;
break;
case 0x72 :w++;//向右
Num = &target_angle.pitch;
add_flag=0;
return 1;
break;
case 0x66 :w++;//向前
Num = &target_angle.roll;
add_flag=0;
return 1;
break;
case 0x62 :w++;//向后
Num = &target_angle.roll;
add_flag=0;
return -1;
break;
case 0x75 :w++;//高度
Num = &target_height;
add_flag=1;
return 1;
case 0x64 :w++;//高度
Num = &target_height;
add_flag=1;
return -1;
break;
case 0x19 :w++;//急停
Num = NULL;
UAV_stop_flag=1;
break;
case 0x20 :w++;//模式切换
UAV_Flymode=!UAV_Flymode;
Num = NULL;
break;
default:
w=0;
break;
}
return 1;
}
