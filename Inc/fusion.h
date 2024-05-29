#ifndef __FUSION_H__
#define __FUSION_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "Data.h"





/*以下是融合处理后的数，可以直接放入pid*/
extern T_Acc Acc;
 extern T_gyro Gyro;
 extern T_angle Angle;
 extern T_coor  Coor;//位移
 extern T_rate  Rate;//速度
 extern double height;//高度值
#ifdef __cplusplus
}
#endif
#endif