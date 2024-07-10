#ifndef __REMOTE_H__
#define __REMOTE_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32h7xx_hal.h"
#include "Data.h"
int8_t Remote_flag(int8_t ch);
void Remote_Updata(int8_t ch);
void Remote_openmv_Updata(uint8_t ch);
extern uint8_t Remote_connectcheck;
extern uint8_t UAV_Flymode;//uav模式选择，0：自主悬停，1：遥控控制（3.测试模式）
extern T_angle target_angle;//获取的目标角度
extern double power;//期望高度
extern uint8_t Remote_hover_flag;
extern double t_height;
extern double t_coodx;
extern double t_coody;
#ifdef __cplusplus
}
#endif

#endif