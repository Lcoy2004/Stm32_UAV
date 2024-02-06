#ifndef __CHRONOSCOPE_H
#define __CHRONOSCOPE_H
void TIM1_Int_Init(void);//定时器1
void Clock1_Start();
u16 Clock1_End();
void TIM4_Int_Init(void);
void Clock2_Start();//定时器4，目前无用
u16 Clock2_End();
#endif