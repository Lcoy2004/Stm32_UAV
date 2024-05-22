#ifndef __KALMAN_H__
#define __KALMAN_H__

#ifdef __cplusplus
extern "C" {
#endif
typedef struct 
{
 float New_Pk;//初始值可以为0
 float Last_Pk;//初始值不能为0
float R;//超参数，自己调
float Q;//超参数，自己调
float Kg;
float output;

}K_Filter;

void kalman_filter(K_Filter *kfc,float measure);

#ifdef __cplusplus
}
#endif
#endif