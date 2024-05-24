#include "Motor.h"
#include "Control.h"
#include "main.h"
double data_limit(double data, double toplimit, double lowerlimit);


int8_t Motor_update(double Motor_roll,double Motor_pitch,double Motor_height,double Motor_yaw )
{
double motor1,motor2,motor3,motor4;
motor1=(Motor_Vmin+Motor_roll+Motor_pitch+Motor_height-Motor_yaw);
motor2=(Motor_Vmin+Motor_roll-Motor_pitch+Motor_height+Motor_yaw);
motor3=(Motor_Vmin-Motor_roll+Motor_pitch+Motor_height+Motor_yaw);
motor4=(Motor_Vmin-Motor_roll-Motor_pitch+Motor_height-Motor_yaw);
//防止超出
motor1=data_limit(motor1,Motor_Vmax,Motor_Vmin);
motor2=data_limit(motor2,Motor_Vmax,Motor_Vmin);
motor3=data_limit(motor3,Motor_Vmax,Motor_Vmin);
motor4=data_limit(motor4,Motor_Vmax,Motor_Vmin);
/*下面motor输出*/
    

/*Motor End*/
return UAVNormal;

}

int8_t Motor_init(void)
{

return UAVNormal;

}



 double data_limit(double data, double toplimit, double lowerlimit)
{
  if(data > toplimit)  data = toplimit;
  else if(data < lowerlimit) data = lowerlimit;
    return data;
}