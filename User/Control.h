#ifndef  __CONTROL_H__
#define __CONTROL_H__
void Control_Motor(float t_yaw,float t_pitch,float t_roll,float t_height,float dt);
void Control_fly();
void Control_stop();//紧急停桨;


#endif