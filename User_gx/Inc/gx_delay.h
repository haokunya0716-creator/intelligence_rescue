//
// Created by gaoxu on 2026/2/8.
//

#ifndef BALANCE_CAR_GX_DELAY_H
#define BALANCE_CAR_GX_DELAY_H
/*
 * 本模块依赖 TIM1 提供 1 MHz 的基础计时。
 *
 * 使用 gx_GetUs() 之前，必须在系统初始化阶段调用：
 *
 *     HAL_TIM_Base_Start_IT(&htim1);
 *
 * 当前 TIM1 的计数频率为 1 MHz，每个计数对应 1 us；TIM1 的
 * 自动重装载值为 65535，更新中断用于软件扩展 16 位计数器。
 */
#include "stdint.h"
uint64_t gx_GetUs(void);//微秒级gettick
uint32_t Return_delay(void);
#endif //BALANCE_CAR_GX_DELAY_H
