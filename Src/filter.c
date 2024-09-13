#include <math.h>  
#include "filter.h"
#define LIMIT( x,min,max ) ( ((x) < (min)) ? (min) : ( ((x) > (max))? (max) : (x) ) )
#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *3.14f *(t) ) ) ) *( (in) - (out) ))	//一阶低通滤波
#define safe_div(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator)/(denominator)) )
// 计算一阶低通滤波器的alpha值  
double calculate_alpha(double hz, double t)
 {  
    return 1.0 / (1.0 + 1.0 / (2.0 * 3.1415926 * hz * t));  
}
// 一阶低通滤波器函数  
double Filter_low_pass_filter(double in, double *out, double hz, double t)
 {  
    double alpha = calculate_alpha(hz, t);  
    *out = alpha * in + (1.0 - alpha) * (*out);  
    return *out;  
}
double Filter_limit(double data, double toplimit, double lowerlimit)
{
  if(data > toplimit)  data = toplimit;
  else if(data < lowerlimit) data = lowerlimit;
    return data;
}
void Filter_1(double base_hz,double gain_hz,double dT,double in,_filter_1_st *f1)   //动态调整滤波截止频率的一阶滤波
{
	LPF_1_(gain_hz,dT,(in - f1->out),f1->a); //低通后的变化量

	f1->b = pow(in - f1->out,2);  //求一个数平方函数

	f1->e_nr = LIMIT(safe_div(pow(f1->a,2),((f1->b) + pow(f1->a,2)),0),0,1); //变化量的有效率，LIMIT 将该数限制在0-1之间，safe_div为安全除法
	
	LPF_1_(base_hz *f1->e_nr,dT,in,f1->out); //低通跟踪
}