#include "stm32f10x.h" 
#include "ESP.h"
int num;
float sum;
extern float t_yaw;
extern float t_pitch;
extern float t_roll;
extern uint16_t t_height;

char ReceiveVis(void)
{
	if(ESP_GetRxFlag()==1)
	{
		return Vis;
		//p£º¼±Í£ l£º×ó b£ººó f£ºÇ° r:ÓÒ u£ºÉÏÉý
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

uint8_t ReceiveNum_Gettarget(void)
{
	int8_t i;
for(i=0;i<5;i++)
{
switch (ReceiveVis())
{
case 'p':
	return 0;
	break;
case 'l':  t_pitch= ReceiveNum();
    break;
case 'b':  t_roll= -ReceiveNum();
    break;
case 'f':  t_roll= ReceiveNum();
    break;
case 'r':  t_pitch= -ReceiveNum();
    break;
case 'u': t_height=(uint16_t)ReceiveNum();
    break;
default:
    return 0;
	break;
}
}
return 1;

}