#ifndef  __CONTROL_H
#define __CONTROL_H
#include "pid.h"
void Control_Motor(float t_yaw,float t_pitch,float t_roll,float t_height,float dt);
void Control_fly();
void Control_stop();//紧急停桨;

//预期目标
extern float t_yaw;
extern float t_pitch;
extern float t_roll;
extern float t_height;

//

#endif