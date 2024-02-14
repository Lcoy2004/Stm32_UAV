#ifndef __ESP_H
#define __ESP_H

#include <stdio.h>

extern int HexNum;
extern char Vis;

void ESP_Init(void);
void ESP_SendByte(uint8_t Byte);
void ESP_SendArray(uint8_t *Array, uint16_t Length);
void ESP_SendString(char *String);
void ESP_SendNumber(uint8_t Number, uint8_t Length);
void ESP_Printf(char *format, ...);

uint8_t ESP_GetRxFlag(void);
uint8_t ESP_GetRxData(void);

#endif
