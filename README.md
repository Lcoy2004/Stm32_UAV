# Stm32_UAV

#### 介绍
使用Stm32独立写的飞控程序。这一次是升级版~（building）

#### 相关mcu,imu和相关处理器
1. WT9011G4K
2.Stm32H750VET6
3.Lora

#### 参与
我们做出一个可以用自制遥控器控制的无人机，能够实现四向移动，上下移
动，方向旋转，保持悬停，特殊情况自动降落，自我姿态矫正的功能；可以在遥
控器端更改无人机的 PID 值；有一个可以识别并捕捉颜色的镜头，同时搭上舵
机，镜头可以随物品的移动而移动。
我们成功利用 STM32H750VBT6 做主控完成了无人机的搭建，开发了主控板
的飞控程序和遥控器（利用 STM32F407RCT6）的通信遥控程序，同时还有
OPENMV 的图像捕捉程序。
飞控程序利用光流模块测量与地面距离，使用陀螺仪捕获无人机姿态并利用
PID 对无人机进行矫正与控制，BMP 气压计模块进一步测量与地面距离，并接
收来自遥控器的信号，最终输出 PWM 波控制无刷电机的转动。通信遥控器利用
芯片 ADC 读取摇杆的状态，以 LORA 为通信模块对无人机发出各种指令；


左边摇杆：控制无人机的四向移动，向前后左右拨动摇杆，会向无人机发射
目标倾斜角度，无人机将会分别向前、后、左、右平移移动。
右边摇杆：上下拨动控制无人机高度，分别给无人机发射累加高度和累减高
度的指令，从而让无人机进行升降；左右拨动控制无人机 z 轴的偏移角度，亦是
通过累加累减角度的方式让无人机旋转不同角度。
无人机 z 轴偏转角度如下图 1-2 中 1 轴转向所示。
图 1-2 无人机三轴转向示意图
2
1.1.2 姿态稳定
可利用 BMP 气压计、九轴陀螺仪、光流计等传感器模拟出飞机姿态和高度，
通过 PID 算法，控制无刷电机的转速，从而在干扰的情况下保持机身稳定。调试
过程如图 1-3 所示。
图 1-3 飞行姿态稳定
1.1.3 识别捕捉颜色
使用 OPENMV 识别特定颜色的物体识别和跟踪。系统能准确捕捉目标，通
过舵机实现精准镜头跟随，确保视角稳定并有效追踪目标物体的移动。考虑到
颜色对识别结果的影响，因此我们选择了生活中较为罕见的颜色作为最终目标
物体的识别。这样做可以有效减少环境中其他色彩对系统识别的干扰，提高识
别的准确性和稳定性。OPENMV 如图 1-4 所示。![输入图片说明](MDK-ARM/%E5%9B%BE%E7%89%87.png)


