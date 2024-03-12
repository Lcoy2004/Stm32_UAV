#include "stm32f10x.h" 
#include "ESP.h"
#include "Serial.h"
int num;
float sum;
extern float t_yaw;
extern float t_pitch;
extern float t_roll;
extern float t_height;
int TEMP;

char ReceiveVis(void)
{
	if(ESP_GetRxFlag()==1)
	{
		return Vis;
		//p：停  F:前 B:后 L：左 R:右 U：上升
	}
}

uint8_t ReceiveNum(void)
{
	if(ESP_GetRxFlag()==1)
	{
		num=HexNum;
		sum=(float)num;
		return HexNum;
	}
}

void ReceiveNum_Gettarget(void)
{
	if(ESP_RxFlag==1)
	{
		//Serial_Printf("%d ", TEMP);

	//int8_t i;
	//for(i=0;i<5;i++)
	switch (ReceiveVis())
		{
		case 'p':break;
		case 'L':  t_pitch= -(float)HexNum;break;
			//Serial_Printf("%d ", -TEMP);break;
		case 'B':  t_roll= -(float)HexNum;break;
			//Serial_Printf("%d ",- TEMP);break;
		case 'F':  t_roll= (float)HexNum;break;
		case 'R':  t_pitch= (float)HexNum;break;
			//Serial_Printf("%d ", TEMP);break;
		case 'U': t_height=(float)HexNum;break;
			//Serial_Printf("%d ", TEMP);break;
		case 'S':Serial_Printf("%d \n", HexNum);break;
		}
	}
//
//return 1;

}