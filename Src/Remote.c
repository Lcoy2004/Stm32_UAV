#include "Remote.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "Data.h"

T_angle target_angle;//获取的目标角度
double target_height;//获取的期望高度
double t_height;//送入pid的期望高度
double t_coodx;
double t_coody;
/**
 * @brief 从遥控端获得期望模式：悬停/遥控控制
 * 
 * @return int8_t 
 */
int8_t UAV_Flymode;//uav模式选择，0：自主悬停，1：遥控控制（3.测试模式）
int8_t Remote_Modeget()
{



    return UAVNormal;
}

/**
 * @brief 获取期望目标
 * 
 * @return int8_t 
 */
int8_t Remote_targetget()
{


return UAVNormal;

}
/**
 * @brief 判断是否连接成功,是否断连。成功返回UAVNormal，断连就UAVError
 * 
 * @return int8_t 
 */
int8_t Remote_connectcheck()
{


return UAVNormal;
}
/**
 * @brief 飞行急停
 * 
 */
void Remote_flyreset()
{

}
/**
 * @brief pid参数调试。修改成功返回UAVNormal，不然就UAVError
 * 
 * @return int8_t 
 */
int8_t Remote_Test_pidmodify()
{


    return UAVNormal;
}