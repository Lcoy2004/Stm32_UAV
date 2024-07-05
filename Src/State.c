#include "State.h"
#include "stm32h7xx_hal.h"
#include "Remote.h"
#include "main.h"
#include "Motor.h"
#include "Data.h"
#include "tim.h"
int8_t current_state;
static int8_t next_state;
static int8_t previous_state;
int8_t land_flag;//失联计时标志
int8_t State_loop()
{
    State_monitering();
switch (current_state)
{
case UAVstart:
    State_start();
    break;
case UAVlanding :
     State_landing();
    break;
    case UAVtakeoff:
   State_takeoff();
    break;
    case UAVstop:
    State_stop();
    break;
    case UAVautofly:
    State_autofly();
    break;
    case UAVremotefly:
    State_remotefly();
    break;
default:
    State_stop();
    return UAVError;
    break;
    
}
if(next_state!=current_state)
{
previous_state=current_state;//为避免重复，上一个状态记到不重复的状态为止
}
current_state=next_state;
State_modechange();//需要监测函数
return UAVNormal;
}

void State_start()
{
     Motor_setspeed1(Motor_Vmin);
     Motor_setspeed2(Motor_Vmin);
     Motor_setspeed3(Motor_Vmin);
     Motor_setspeed4(Motor_Vmin);
  
        next_state=UAVtakeoff;//预备起飞状态
}

void State_landing()
{
if((previous_state==UAVremotefly||previous_state==UAVautofly||previous_state==UAVlanding))//在3种情况下进入降落程序
{
    if((height<Landing_Max_Height)&&(target_height<Landing_Max_Height))
    {
        t_height-=0.05;
        next_state=UAVlanding;//循环进入下降程序，直至下降
    }else if(height<35&&t_height<40)//下降完成
    {
       next_state=UAVstart;//等待下次起飞
       HAL_TIM_Base_Stop_IT(&htim7);//关闭pid
    }else//先下降，预定高度过低
    {
     next_state=UAVlanding;//循环进入下降程序，直至下降
    }
}else//其他情况回到上一个状态
{
    next_state=previous_state;
}

}

void State_takeoff()
{
if(previous_state==UAVstart||previous_state==UAVtakeoff)
{
    if(target_height>Takingoff_Min_Hegiht)
    {
         HAL_TIM_Base_Start_IT(&htim7);//开启pid
         t_height=150;//预定起飞高度定死
         next_state=UAVtakeoff;
    }
    else if(target_height<Takingoff_Min_Hegiht)
    {
    t_height=0;
    Motor_setspeed1(Motor_Vmin);
     Motor_setspeed2(Motor_Vmin);
    Motor_setspeed3(Motor_Vmin);
     Motor_setspeed4(Motor_Vmin);
     next_state=UAVstart;//进入未启动状态
    }else if((t_height-height<20)&&(t_height-height>0)&&(height>130))//起飞完毕
    {
       next_state=UAVautofly;//进入自主悬停状态
    }
else 
{
    next_state=previous_state;//其余进入起飞状态时回到上一个状态
}
}
}
void State_modechange()//需要监测函数
{
if(UAV_Flymode==0&&(previous_state==UAVautofly||previous_state==UAVremotefly))
{
next_state=UAVautofly;
}else if(UAV_Flymode==1&&(previous_state==UAVautofly||previous_state==UAVremotefly))
{
next_state=UAVremotefly;
}
}

void State_stop()
{
    HAL_TIM_Base_Stop_IT(&htim7);//关闭pid
     Motor_setspeed1(Motor_Vmin);
     Motor_setspeed2(Motor_Vmin);
    Motor_setspeed3(Motor_Vmin);
     Motor_setspeed4(Motor_Vmin);
  next_state=UAVstop;//定死了，不能再次起飞

}
void State_autofly()
{
 t_height=target_height;
 if(previous_state!=UAVautofly)//记录第一次UAV悬停时的位移
 {
 t_coodx=Coor.x;
 t_coody=Coor.y;
 }
 if((height<Landing_Max_Height)&&(target_height<Landing_Max_Height))//判断是否降落
    {
        next_state=UAVlanding;
    }else{
    next_state=UAVautofly;
    }
   //默认下次还是进入悬停，若有更改在Controlmonitering里面更改
}
void State_remotefly()
{
    t_height=target_height;
     if((height<Landing_Max_Height)&&(target_height<Landing_Max_Height))//判断是否降落
    {
        next_state=UAVlanding;
    }else{
    next_state=UAVremotefly;
    }
    //默认下次还是进入遥控，若有更改在Controlmonitering里面更改
}
void State_monitering()//监视状态，临时改变状态;
{
if(Angle.pitch>85||Angle.roll>85)
{
    current_state=UAVstop;//俯仰角过大，判定炸机
}else if(Remote_connectcheck==UAVError)
{
    if(previous_state==UAVremotefly||previous_state==UAVautofly)
    { //这里开启计时7s
    HAL_TIM_Base_Start_IT(&htim13);
      if(land_flag==1)//计时7秒
        {
          current_state=UAVtakeoff;//计时开始降落
        }
      else if(land_flag==0)
        {
            static int8_t i=0;
            if(i<1)
            {
            HAL_TIM_Base_Start_IT(&htim13);
            i++;
            }
          next_state=UAVautofly;//进入悬停
        } 
    }else
    {
        next_state=previous_state;//起飞和降落时断联先把动作做完
        
    }
  }
}
