#include <math.h>  
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