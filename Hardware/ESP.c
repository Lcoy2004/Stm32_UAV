#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include "Receive.h"

uint8_t ESP_RxData;
uint8_t ESP_RxFlag;

int HexNum;
char Vis;


void ESP_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;//
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



uint8_t ESP_GetRxData(void)
{
	return ESP_RxData;
}


uint8_t ESP_GetRxFlag(void)
{
	if(ESP_RxFlag==1)
	{
		ESP_RxFlag=0;
		return 1;
	}
	else 
		return ESP_RxFlag;
}
int Pow(int x,int y)
{
	while(y--)
	{
		x*=x;
	}
	x/=x;
	return x;
}

void USART2_IRQHandler(void)
{
	static int RxState=0;
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		int RxData=USART_ReceiveData(USART2);
		//HexNum=RxData;
		if(RxState==0)
		{
			HexNum=0;
			if(RxData==0)
			{
				Vis='p';
			}
			else if(RxData==1)
			{
				Vis='L';
			}
			else if(RxData==2)
			{
				Vis='B';
			}
			else if(RxData==3)
			{
				Vis='F';
			}
			else if(RxData==4)
			{
				Vis='R';
			}
			else if(RxData==5)
			{
				Vis='U';
			}
			else if(RxData==6)
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
				RxState++;
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
		TEMP=HexNum;
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
}
