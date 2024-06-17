#ifndef __STATE_H__
#define __STATE_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32h7xx_hal.h"
extern int8_t current_state;
int8_t State_loop();
void State_start();

void State_landing();
void State_takeoff();

extern int8_t land_flag;//失联计时标志
#ifdef __cplusplus
}
#endif
#endif