#ifndef __REMOTE_H__
#define __REMOTE_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32h7xx_hal.h"
#include "Data.h"
void Remote_flyreset();
int8_t Remote_Test_pidmodify();
int8_t Remote_Modeget();
int8_t Remote_connectcheck();
int8_t Remote_targetget();
void Remote_flyreset();
extern int8_t UAV_Flymode;//uav模式选择，0：自主悬停，1：遥控控制（3.测试模式）
extern T_angle target_angle;//获取的目标角度
extern double target_height;//期望高度
extern double t_height;
extern double t_coodx;
extern double t_coody;
#ifdef __cplusplus
}
#endif

#endif