#include "myMath.h"
#include <math.h>

//私有变量区
const float M_PI = 3.1415926535f;
const float RtA = 57.296f;
const float AtR = 0.0174f;
const float Gyro_G = 0.03051756f * 2;          
const float Gyro_Gr = 0.0005326f * 2;    
const float PI_2 = 1.570796f;

/******************************************************************************
  * 函数名称：sin
  * 函数描述：返回正弦值
  * 输    入：float x:角度值
  * 输    出：void
  * 返    回：正弦值
  * 备    注：null
  *    
  *
******************************************************************************/
float sine(float x)          
{
    const float Q = 0.775;
    const float P = 0.225;
    const float B =  4 / M_PI;  
    const float C = -4 / (M_PI * M_PI);
    float y = B * x + C * x * fabs(x); 
    return (Q * y + P * y * fabs(y));
}

/******************************************************************************
  * 函数名称：cosin
  * 函数描述：计算余弦值
  * 输    入：float x:角度
  * 输    出：
  * 返    回：余弦值 
  * 备    注：cos(x)=sin(M_PI / 2 + x)=sin(M_PI / 2 - x)  
  *    
  *
******************************************************************************/
float cosin(float x)
{
    return sine(x + M_PI / 2);
}

/******************************************************************************
  * 函数名称：arctan
  * 函数描述：反正切函数
  * 输    入：反正切数据
  * 输    出：void
  * 返    回：反正切值
  * 备    注：反正切麦克劳林展开式 阶数越高，值越准确70°以内是准确的  
  *    
  *
******************************************************************************/
float arctan(float x)  //  (-1 , +1)    6? ?? 0.002958 
{
    float t = x;
    float result = 0;
    float X2 = x * x;
    unsigned char cnt = 1;
    do
    {
        result += t / ((cnt << 1) - 1);
        t = -t;
        t *= X2;
        cnt++;
    }while(cnt <= 6);
    
    return result;
}

/******************************************************************************
  * 函数名称：arcsin
  * 函数描述：反正弦函数
  * 输    入：float x:反正弦数据
  * 输    出：void
  * 返    回：反正弦数据
  * 备    注：反正弦麦克劳林展开式 -1 < x < +1     42°以内是准确的    
  *    
  *
******************************************************************************/
float arcsin(float x)
{
    float d = 1;
    float t = x;
    unsigned char cnt = 1;
    float result = 0;    
    float X2 = x * x;
    
    if (x >= 1.0f) 
    {
        return PI_2;
    }
    if (x <= -1.0f) 
    {
        return -PI_2;
    }
    do
    {
        result += t / (d * ((cnt << 1) - 1));
        t *= X2 * ((cnt << 1) - 1);//
        d *= (cnt << 1);//2 4 6 8 10 ...
        cnt++;
    }while(cnt <= 6);

    return result;
}

/******************************************************************************
  * 函数名称：Q_rsqrt
  * 函数描述：快速计算 1 / Sqrt(x) 
  * 输    入：float number:要计算的数据
  * 输    出：void
  * 返    回：1 / Sqrt(x) 
  * 备    注：null
  *    
  *
******************************************************************************/
float Q_rsqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;
 
    x2 = number * 0.5F;
    y  = number;
    i  = *(long*) &y;                      
    i  = 0x5f3759df - ( i >> 1 );               
    y  = *(float*) &i;
    y  = y * (threehalfs - (x2 * y * y ));   // 1st iteration （第一次牛顿迭代）
    return y;
}


/******************************************************************************
  * 函数名称：data_limit
  * 函数描述：数据限幅
  * 输    入：float data:要操作的数据 
              float toplimit:上限
              float lowerlimit:下限
  * 输    出：
  * 返    回： 
  * 备    注：    
  *    
  *
******************************************************************************/
float data_limit(float data, float toplimit, float lowerlimit)
{
  if(data > toplimit)  data = toplimit;
  else if(data < lowerlimit) data = lowerlimit;
    return data;
}

//**************************实现函数********************************************
//*函数原型:    rad(flaot angle)
//*功　　能:    角度转化为弧度
//输入参数：    角度
//输出参数：    弧度
//*******************************************************************************/
float Rad(float angle)
{
    return angle * AtR ;
}

//**************************实现函数********************************************
//*函数原型:    AT(float angle)
//*功　　能:    弧度转化为角度
//输入参数：    弧度
//输出参数：    角度
//*******************************************************************************/
float AT(float angle)
{

return angle*RtA;

}
