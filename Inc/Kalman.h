#ifndef __KALMAN_H__
#define __KALMAN_H__

#ifdef __cplusplus
extern "C" {
#endif
typedef struct 
{
 double New_Pk;//初始值可以为0
  double Last_Pk;//初始值不能为0
 double R;//超参数，自己调
 double Q;//超参数，自己调
 double Kg;
 double output;

}K_Filter;

void kalman_filter(K_Filter *kfc, double measure);

#ifdef __cplusplus
}
#endif
#endif