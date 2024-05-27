#include "Kalman.h"

//@para：measure：传感器测量值
//@para：kfc:传入指针（纪念星期四！）
void kalman_filter(K_Filter *kfc, double measure)
{
//协方差更新
kfc->New_Pk=kfc->Last_Pk+kfc->Q;
//更新Kalman增益
kfc->Kg=kfc->New_Pk/(kfc->New_Pk+kfc->R);
//更新输出
kfc->output=kfc->output+kfc->Kg*(measure-kfc->output);
kfc->Last_Pk=(1 - kfc->Kg)*kfc->New_Pk;
}