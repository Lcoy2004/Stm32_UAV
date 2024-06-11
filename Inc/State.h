#ifndef __STATE_H__
#define __STATE_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32h7xx_hal.h"
int8_t State_loop();
void State_start();

void State_landing();
void State_takeoff();


#ifdef __cplusplus
}
#endif
#endif