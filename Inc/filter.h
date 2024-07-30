#ifndef __FILTER_H__
#define __FILTER_H__

#ifdef __cplusplus
extern "C" {
#endif
typedef struct 
{
double a;
double b;
double e_nr;
double out;
}_filter_1_st;
double Filter_low_pass_filter(double in, double *out, double hz, double t);
double Filter_limit(double data, double toplimit, double lowerlimit);
void Filter_1(double base_hz,double gain_hz,double dT,double in,_filter_1_st *f1) ;  //动态调整滤波截止频率的一阶滤波
#ifdef __cplusplus
}
#endif
#endif