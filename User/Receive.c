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
		//p???б└?? l??б┴ио b???ио f???бу r:?? u??????
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
case 'L':  t_pitch= ReceiveNum();
    break;
case 'B':  t_roll= -ReceiveNum();
    break;
case 'F':  t_roll= ReceiveNum();
    break;
case 'R':  t_pitch= -ReceiveNum();
    break;
case 'U': t_height=(uint16_t)ReceiveNum();
    break;
default:
    return 0;
	break;
}
}
return 1;

}