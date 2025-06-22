

# CMSIS-DSP 示例测试套件

## 项目简介

本项目是CMSIS-DSP库提供的官方测试套件，用于验证数字信号处理模块的正确性。套件包含基础数学运算、复数运算、滤波、矩阵操作、快速数学函数、统计分析等多种功能的测试案例。

## 项目结构

```
Drivers/CMSIS/DSP/DSP_Lib_TestSuite/
├── Common                  # 公共组件与测试框架
├── RefLibs                 # 参考实现库
├── DspLibTest_FVP          # Fast Models虚拟平台测试
├── DspLibTest_SV_FVP       # 安全验证测试
├── DspLibTest_SV_MPS2     # MPS2平台测试
└── Examples                # 示例项目
```

## 主要功能模块

### 基础数学测试
- 向量加减乘除
- 数值缩放与偏移
- 数值绝对值与取反

### 复数运算测试
- 复数共轭
- 复数点乘
- 复数模运算

### 滤波功能测试
- FIR滤波器
- IIR滤波器
- 自适应滤波器
- 卷积与相关性分析

### 矩阵运算测试
- 矩阵加减
- 矩阵乘法
- 矩阵转置
- 矩阵缩放
- 矩阵求逆

### 快速数学函数测试
- 正弦与余弦计算
- 平方根计算

### 统计分析测试
- 最大/最小值检测
- 均值、功率、RMS计算
- 方差与标准差分析

## 使用示例

### 基础数学运算
```c
// 浮点加法测试
#define JTEST_ARM_ADD_TEST(suffix)              \
JTEST_DEFINE_GROUP(add_tests)
```

### 复数模运算
```c
// 浮点复数模测试
#define JTEST_ARM_CMPLX_MAG_TEST(suffix, comparison_interface) \
...
JTEST_DEFINE_GROUP(cmplx_mag_tests)
```

### FIR滤波器测试
```c
// 测试FIR滤波器
#define JTEST_ARM_FIR_TEST(suffix, config_suffix, output_type) \
...
JTEST_DEFINE_GROUP(fir_tests)
```

## 测试框架说明

采用JTEST框架进行模块化测试，每个模块定义格式：
```c
JTEST_DEFINE_GROUP(module_name)
```

测试函数采用统一命名模式：
```c
#define JTEST_ARM_XXX_TEST(suffix) \
...
```

## 参考实现

所有测试都有对应的参考实现，如：
```c
// 参考加法实现
void ref_add_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  float32_t * pDst,
  uint32_t blockSize)
```

## 平台支持

支持多种Cortex-M系列处理器：
- ARMv8-M Baseline (ARMv8MBL)
- ARMv8-M Mainline (ARMv8MML)
- Cortex-M0 (ARMCM0)
- Cortex-M3 (ARMCM3)
- Cortex-M4 (ARMCM4)
- Cortex-M7 (ARMCM7)
- Cortex-A5 (ARMCA5)

## 开发环境

支持主流编译器：
- ARMCC
- ARMClang
- GCC
- IAR

## 特性

- 完整的DSP功能测试覆盖
- 支持浮点与定点运算测试
- 提供详细的测试数据定义
- 多平台验证支持
- 包含Python包装器接口
- 支持安全与非安全上下文切换测试

## 开发者资源

包含辅助函数：
```c
// SNR计算
float arm_snr_f32(float *pRef, float *pTest, uint32_t buffSize)

// 守护位处理
void arm_provide_guard_bits_q15(q15_t * input_buf, uint32_t blockSize, uint32_t guard_bits)

// 类型转换
void arm_float_to_q14(float *pIn, q15_t *pOut, uint32_t numSamples)
```

## 测试执行

主测试程序：
```c
int main(void) {
    debug_init();
    JTEST_RUN_ALL_TESTS();
    return 0;
}
```

## 许可协议

本项目采用开源许可协议，具体信息请查看项目根目录的LICENSE文件。

## 文档更新状态

当前文档基于代码结构自动生成，确保与最新代码保持同步。