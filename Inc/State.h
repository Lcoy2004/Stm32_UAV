#ifndef __STATE_H__
#define __STATE_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32h7xx_hal.h"
#define Landing_Max_Height 50
#define Takingoff_Min_Hegiht 50

extern int8_t current_state;
extern uint8_t UAV_stop_flag;
int8_t State_loop();
void State_start();

void State_landing();
void State_takeoff();

extern int8_t land_flag;//失联计时标志
#ifdef __cplusplus
}
#endif
#endif