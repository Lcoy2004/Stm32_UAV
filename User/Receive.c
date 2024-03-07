#include "stm32f10x.h" 
#include "ESP.h"
#include "Serial.h"
int num;
float sum;
extern float t_yaw;
extern float t_pitch;
extern float t_roll;
extern float t_height;

char ReceiveVis(void)
{
	if(ESP_GetRxFlag()==1)
	{
		return Vis;
		//p：停  F:前 B:后 L：左 R:右 U：上升
	}
}

float ReceiveNum(void)
{
	if(ESP_GetRxFlag()==1)
	{
		return (float)HexNum;
	}
}

void ReceiveNum_Gettarget(void)
{
	
	if(ESP_GetRxFlag()==1)
	{
		if (ReceiveVis()=='p')
		{
			
		}
		else if(ReceiveVis()=='F')
			t_roll= ReceiveNum();
		else if(ReceiveVis()=='B')
			t_roll= -ReceiveNum();
		else if(ReceiveVis()=='L')
			t_pitch= -ReceiveNum();
		else if(ReceiveVis()=='R')
			t_pitch= ReceiveNum();
		else if(ReceiveVis()=='U')
			t_height= ReceiveNum();
	}
//return 1;
/*case 'L':  t_pitch= -ReceiveNum();
    /[表情];
case 'B':  t_roll= -ReceiveNum();
    /[表情];
case 'F':  t_roll= ReceiveNum();
    /[表情];
case 'R':  t_pitch= ReceiveNum();
    /[表情];
case 'U': t_height=ReceiveNum();*/

}