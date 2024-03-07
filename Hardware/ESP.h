#ifndef __ESP_H
#define __ESP_H

#include <stdio.h>

extern int HexNum;
extern char Vis;
extern uint8_t ESP_RxFlag;

void ESP_Init(void);

uint8_t ESP_GetRxFlag(void);
uint8_t ESP_GetRxData(void);

#endif
