#ifndef  __Kalman_H
#define __Kalman_H
#include "stm32f10x.h"
typedef struct 
{
 float New_Pk;//初始值可以为0
 float Last_Pk;//初始值不能为0
float R;//超参数，自己调
float Q;//超参数，自己调
float Kg;
float output;

}K_Filter;

void kalman_filter(K_Filter *kfc,float measure);

#endif