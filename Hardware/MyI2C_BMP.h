#include "stm32f10x.h"
#ifndef __MYI2C_BMP_H
#define __MYI2C_BMP_H
                  // Device header

void MyI2C_BMP_Init(void);
void MyI2C_BMP_Start(void);
void MyI2C_BMP_Stop(void);
void MyI2C_BMP_SendByte(uint8_t Byte);
uint8_t MyI2C_BMP_ReceiveByte(void);
void MyI2C_BMP_SendAck(uint8_t AckBit);
uint8_t MyI2C_BMP_ReceiveAck(void);

#endif
