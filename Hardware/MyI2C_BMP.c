#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "BMP280.h"
void MyI2C_BMP_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOA, GPIO_Pin_0, (BitAction)BitValue);
	Delay_us(1);
}

void MyI2C_BMP_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOA, GPIO_Pin_1, (BitAction)BitValue);
	Delay_us(1);
}

uint8_t MyI2C_BMP_R_SDA(void)
{
	uint8_t BitValue;
	BitValue = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);
	Delay_us(1);
	return BitValue;
}

void MyI2C_BMP_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	//bmp280
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1);
	//BMP280 SCL接PA0， SDA接PA1
}

void MyI2C_BMP_Start(void)
{
	MyI2C_BMP_W_SDA(1);
	MyI2C_BMP_W_SCL(1);
	MyI2C_BMP_W_SDA(0);
	MyI2C_BMP_W_SCL(0);
}

void MyI2C_BMP_Stop(void)
{
	MyI2C_BMP_W_SDA(0);
	MyI2C_BMP_W_SCL(1);
	MyI2C_BMP_W_SDA(1);
}

void MyI2C_BMP_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)
	{
		MyI2C_BMP_W_SDA(Byte & (0x80 >> i));
		MyI2C_BMP_W_SCL(1);
		MyI2C_BMP_W_SCL(0);
	}
}

uint8_t MyI2C_BMP_ReceiveByte(void)
{
	uint8_t i, Byte = 0x00;
	MyI2C_BMP_W_SDA(1);
	for (i = 0; i < 8; i ++)
	{
		MyI2C_BMP_W_SCL(1);
		if (MyI2C_BMP_R_SDA() == 1){Byte |= (0x80 >> i);}
		MyI2C_BMP_W_SCL(0);
	}
	return Byte;
}

void MyI2C_BMP_SendAck(uint8_t AckBit)
{
	MyI2C_BMP_W_SDA(AckBit);
	MyI2C_BMP_W_SCL(1);
	MyI2C_BMP_W_SCL(0);
}

uint8_t MyI2C_BMP_ReceiveAck(void)
{
	uint8_t AckBit;
	MyI2C_BMP_W_SDA(1);
	MyI2C_BMP_W_SCL(1);
	AckBit = MyI2C_BMP_R_SDA();
	MyI2C_BMP_W_SCL(0);
	return AckBit;
}




