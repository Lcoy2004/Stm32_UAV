#include "stm32f10x.h" 
#include "ESP.h"
int num;
float sum;

uint8_t ReceiveVis(void)
{
	if(ESP_GetRxFlag()==1)
	{
		return Vis;//0：急停 1：左 2：后 3：前 4：右 5：上升高度
		//num=HexNum;
		//sum=(float)num;
		//OLED_ShowChar(1,1, Vis);
		//OLED_ShowHexNum(3,1,sum,3);
	}
}

float ReceiveNum(void)
{
	if(ESP_GetRxFlag()==1)
	{
		
		num=HexNum;
		sum=(float)num;
		//OLED_ShowChar(1,1, Vis);
		//OLED_ShowHexNum(3,1,sum,3);
		return sum;
	}
}