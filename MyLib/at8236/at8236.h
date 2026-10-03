#if 0 /* AT8236 implementation kept for reference; DRV8701 is used now. */
#ifndef AT8236_H
#define AT8236_H

/* AT8236 STM32 HAL 电机控制接口。 */

#include <stdint.h>
#include "main.h"

/**
 * @brief 初始化 AT8236 电机驱动使用的 PWM 输出。
 *
 * 函数会启动驱动中配置的 PWM 通道，并将左右电机先设置为停止状态。
 * 调用本函数前，必须先完成对应 STM32 定时器和 GPIO 复用功能的初始化。
 */
void Motor_Init(void);

/**
 * @brief 使能或禁止 AT8236 的电机输出。
 *
 * @param on 0 表示禁止输出并立即停止两侧电机；非 0 表示允许更新
 *           电机 PWM 输出。
 */
void Motor_Cmd(uint8_t on);

/**
 * @brief 设置左侧电机的速度和方向。
 *
 * @param duty 占空比控制量，范围为 -100.0f 至 +100.0f。正负号的
 *             实际机械方向沿用源 TI 程序中的定义。
 */
void Motor_Set_L(float duty);

/**
 * @brief 设置右侧电机的速度和方向。
 *
 * @param duty 占空比控制量，范围为 -100.0f 至 +100.0f。正负号的
 *             实际机械方向沿用源 TI 程序中的定义。
 */
void Motor_Set_R(float duty);

/**
 * @brief 停止左侧电机。
 *
 * 该函数只将左侧两路 PWM 设置为停止状态，不关闭 PWM 定时器。
 */
void Motor_Stop_L(void);

/**
 * @brief 停止右侧电机。
 *
 * 该函数只将右侧两路 PWM 设置为停止状态，不关闭 PWM 定时器。
 */
void Motor_Stop_R(void);

/**
 * @brief 同时停止左右两侧电机。
 *
 * 该函数只修改 PWM 比较值，不改变输出使能标志。
 */
void Motor_Stop(void);

#endif
#endif /* AT8236 reference code */
