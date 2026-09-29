//
// Created by gaoxu on 2026/2/7.
//

#ifndef BALANCE_CAR_APP_ENCODER_H
#define BALANCE_CAR_APP_ENCODER_H

/* ================= 电机参数配置 ================= */

// 1. 编码器单相原始线数 (对极个数)(注意：STM32开启4倍频计数，实际逻辑中会乘以4)
#define ENCODER_PPR            13.0f

// 2. 电机减速比 (请根据你购买的电机具体参数修改)
// 常见的 MG513 多为 30 或 50；GA25 多为 34 或 75
#define GEAR_RATIO             30.0f

// 3. 轮子直径 (单位: mm)
#define WHEEL_DIAMETER         54.0f

/* ================= 衍生计算宏 (自动计算，无需手动改) ================= */

#define PI                     3.14159265f


// 轮子转一圈，STM32 编码器模式下计数值的变化量 (11*4*30 = 1320)

//#define ENCODER_TOTAL_RESOLUTION  (ENCODER_PPR * 4.0f * GEAR_RATIO)
//下面这个是外部中断的
#define ENCODER_TOTAL_RESOLUTION (2.0f * ENCODER_PPR * GEAR_RATIO)

// 轮子周长 (单位: mm)
#define WHEEL_PERIMETER           (PI * WHEEL_DIAMETER)

// 脉冲数转距离系数 (mm/脉冲): 走多少个脉冲代表走了多少mm
// 物理意义：当前轮子走 1mm 对应的编码器脉冲数 = ENCODER_TOTAL_RESOLUTION / WHEEL_PERIMETER
#define PULSE_TO_DISTANCE_RATIO   (WHEEL_PERIMETER / ENCODER_TOTAL_RESOLUTION)

#include "main.h"

int64_t App_Encoder_GetEncoder_L(void);//读取左编码器的值
int64_t App_Encoder_GetEncoder_R(void);

float App_Encoder_GetPos_L(void);//读取左编码器的位置
float App_Encoder_GetPos_R(void);

float App_Encoder_GetSpeed_L(void);//T法测速，左电机,单位角速度
float App_Encoder_GetSpeed_R(void);

float App_Encoder_GetLinearSpeed_L(void); // 获取左轮线速度 cm/s
float App_Encoder_GetLinearSpeed_R(void); // 获取右轮线速度 cm/s
#endif //BALANCE_CAR_APP_ENCODER_H
