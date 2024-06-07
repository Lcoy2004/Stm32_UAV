#include "Kalman.h"
#include "math.h"
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

  
// 自适应调整过程噪声协方差Q  
void adaptiveProcessNoise(AdaptiveKalmanFilter *filter, double state_prediction, double state_estimate, double process_noise_factor) {  
    double prediction_error = state_prediction - state_estimate;  
    filter->process_cov *= (1 + process_noise_factor * pow(prediction_error, 2));  
    // 这里process_noise_factor是一个小的正数，用于控制Q的适应速度  
}  
  
// 自适应调整测量噪声协方差R（简化的例子，通常需要根据具体情况实现）  
void adaptiveMeasurementNoise(AdaptiveKalmanFilter *filter, double measurement, double state_estimate, double measurement_noise_factor) {  
    double measurement_error = measurement - state_estimate;  
    filter->measurement_cov *= (1 + measurement_noise_factor * pow(measurement_error, 2));  
    // 这里measurement_noise_factor是一个小的正数，用于控制R的适应速度  
}  
  
// 预测步骤  
void predict(AdaptiveKalmanFilter *filter, double process_noise) {  
    // 假设这里有一个简单的状态转移模型，例如state = state + process_noise  
    // 在实际应用中，这将是一个基于你系统模型的复杂函数  
    filter->state += process_noise; // 简化的例子  
    filter->error_cov += filter->process_cov; // 预测误差协方差增长  
}  
  
// 更新步骤  
void Kalman_update(AdaptiveKalmanFilter *filter, double measurement) {  
    // 计算卡尔曼增益  
    filter->kalman_gain = filter->error_cov / (filter->error_cov + filter->measurement_cov);  
      
    // 更新状态估计  
    filter->state = filter->state + filter->kalman_gain * (measurement - filter->state);  
      
    // 更新估计误差协方差  
    filter->error_cov = (1 - filter->kalman_gain) * filter->error_cov;  
      
    // 自适应调整过程噪声和测量噪声 
     //adaptiveProcessNoise(filter, /* 预测的状态 */, filter->state, /* process_noise_factor */);  
     //adaptiveMeasurementNoise(filter, measurement, filter->state, /* measurement_noise_factor */);  
}  