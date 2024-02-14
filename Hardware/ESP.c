#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>

uint8_t ESP_RxData;
uint8_t ESP_RxFlag;

int HexNum;
char Vis;

void ESP_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//串口设置
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//串口设置
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//串口设置
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;//波特率设置
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &USART_InitStructure);
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART2, ENABLE);
}

void ESP_SendByte(uint8_t Byte)
{
	USART_SendData(USART2, Byte);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void ESP_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		ESP_SendByte(Array[i]);
	}
}

void ESP_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		ESP_SendByte(String[i]);
	}
}

uint32_t ESP_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void ESP_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		ESP_SendByte(Number / ESP_Pow(10, Length - i - 1) % 10 + '0');
	}
}

int ESP_fputc(int ch, FILE *f)
{
	ESP_SendByte(ch);
	return ch;
}

void ESP_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	ESP_SendString(String);
}


uint8_t ESP_GetRxData(void)
{
	return ESP_RxData;
}


uint8_t ESP_GetRxFlag(void)
{
	if(ESP_RxFlag==1)
	{
		return 1;
		ESP_RxFlag=0;
	}
	else 
		return ESP_RxFlag;
}

void USART2_IRQHandler(void)
{
	static int RxState=0;
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		int RxData=USART_ReceiveData(USART2);
		if(RxState==0)
		{
			HexNum=0;
			if(RxData==0x31)
			{
				Vis='L';
			}
			else if(RxData==0x32)
			{
				Vis='B';
			}
			else if(RxData==0x33)
			{
				Vis='F';
			}
			else if(RxData==0x34)
			{
				Vis='R';
			}
			else if(RxData==0x35)
			{
				Vis='U';
			}
			else
			{
				Vis='S';
			}
			RxState++;
		}
		else if(RxState==1)
		{
			if(Vis!='U')
			{
				HexNum=RxData;
				RxState--;
				ESP_RxFlag=1;
			}
			else
			{
				HexNum+=RxData*100;
				RxState++;
				ESP_RxFlag=0;
			}
		}
		else if(RxState==2)
		{
			HexNum+=RxData;
			ESP_RxFlag=1;
			RxState=0;
		}
	}
}

