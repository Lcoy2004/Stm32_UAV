/*
 * PID Controller Implementation in C
 * 
 * Created by Joshua Saxby (aka @saxbophone) on 1 Jan, 2016

 * See LICENSE for licensing details.
 */

// protection against multiple includes
#ifndef SAXBOPHONE_PID_H
#define SAXBOPHONE_PID_H

#ifdef __cplusplus
extern "C"{
#endif


    typedef struct pid_calibration {
        /*
         * 存储PID控制器校准PID常数的结构

          *这些用于调整算法

          *取决于应用程序-（换句话说，YMMV…）
         */
        double kp; // 比例增益
        double ki; //积分增益
        double kd; // 微分增益
    } PID_Calibration;


    typedef struct pid_state {
        /*
         * struct PID_State
         * 
         存储PID控制器当前状态的结构。
    *这被用作PID算法函数的输入值
     *返回PID_State结构，反映算法建议的调整。
    *注意：此结构体中的输出字段由PID算法函数设置，并且
     *在实际计算中被忽略
         */
        double actual; // 测量的实际读数
        double target; // 所需的读数
        double time_delta; // 自上次采样/计算以来的时间-应在更新状态时设置
        // 先前计算的实际和目标之间的误差（最初为零）
        double previous_error;
        double integral; // 随时间变化的积分误差总和
        double output; // 由算法计算的修正输出值，以补偿误差
    } PID_State;


    /*
     * 修正后的输出值由算法计算，以补偿误差PID控制器算法的实现
*给定P、I和D值的PID校准以及电流的PID_State
*PID控制器的状态，计算PID控制器的新状态并设置
*补偿算法定义的任何误差的输出状态
     */
    PID_State pid_iterate(PID_Calibration calibration, PID_State state);


#ifdef __cplusplus
} // extern "C"
#endif

// end of header
#endif