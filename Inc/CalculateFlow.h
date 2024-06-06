#ifndef __CALCULATEFLOW_H__
#define __CALCULATEFLOW_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "Data.h"

void CalculateFlow_upixels_Complementary(double dt,double dT,T_gyro gyro);


extern T_rate  flow_Rate;//光流融合的速度值
extern  T_coor  flow_Coor;//光流积分得到的值
extern int16_t flow_x_integral ;
extern int16_t flow_y_integral ;
extern uint16_t ground_distance ;
extern uint8_t valid ;
extern uint8_t tof_confidence ;
#ifdef __cplusplus
}
#endif
#endif