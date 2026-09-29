//
// Created by gaoxu on 2026/2/8.
//

#include "../Inc/gx_delay.h"
#include "tim.h"
#include "Inc/app_usart.h"
#include "Inc/app_button.h"
/*
 *
 * 使用前必须调用 HAL_TIM_Base_Start_IT(&htim1) 启动 TIM1。
 * TIM1 的计数频率必须为 1 MHz，即每个计数对应 1 us。
 * 当前 STM32F407 工程中 TIM1 的配置为 Prescaler=167、ARR=65535，
 * 因此 TIM1 为 16 位、1 MHz 的微秒计时器。
 *
 * TIM1 计数器从 0 计数到 65535 后回到 0，每次更新事件代表
 * 经过 65536 us。程序使用 g_us 保存软件扩展的高位累计值，
 * 将 16 位硬件计数器扩展为连续的 64 位微秒时间戳。
///////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
*/
static volatile uint64_t g_us = 0;//全局us计时
static volatile uint32_t delay_i = 0;

/**
 * @brief 处理定时器更新事件并扩展微秒时间戳。
 *
 * TIM1 是 16 位定时器，计数器从 0 计数到 65535 后回到 0。
 * 每次溢出时，本函数给软件累计值增加 65536 us，从而将硬件的
 * 16 位计数器扩展为 64 位时间戳。
 *
 * @param htim 产生更新事件的定时器句柄。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    // // 检查是否是定时器 3 触发的中断(10ms)
    // if (htim->Instance == TIM3)
    // {
    //     // --- 在这里执行你的任务 ---
    //     App_Button_Proc();//扫描按键
    //     //App_Motor_Proc(); // 例如：执行电机 PID 控制逻辑
    //     // delay_i++;
    // }
    if (htim->Instance == TIM1)
    {
        g_us += 0x10000ULL;        // 16位定时器溢出补偿
    }
}

/**
 * @brief 返回延时计数变量的当前值。
 *
 * 当前文件中 delay_i 没有被定时器回调递增，函数保留原有接口，
 * 便于其他模块继续编译和调用。
 *
 * @return uint32_t 当前 delay_i 的值。
 */
uint32_t Return_delay(void) {
    return delay_i;
}

/**
 * @brief 获取当前的 64 位微秒时间戳。
 *
 * 函数将 TIM1 软件扩展的高位和当前硬件计数器低位相加。读取过程
 * 临时关闭中断，避免读取高位累计值和计数器的过程中被 TIM1 溢出
 * 中断打断，造成时间戳前后不一致。
 *
 * 另外，若读取时 TIM1 已经产生更新标志但对应中断还没有执行，
 * 函数会临时补上一次 65536 us，并重新读取计数器，避免在溢出
 * 边界返回一个倒退的时间值。
 *
 * @return uint64_t 当前微秒时间戳，单位为 us。
 */
uint64_t gx_GetUs(void)
{
    uint64_t base;
    uint16_t cnt;

    __disable_irq();

    base = g_us;
    cnt  = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);

    /*
     * 若此刻已经溢出但中断还没有执行，则在关中断状态下直接
     * 清除 UIF 并把这次溢出正式记入 g_us。不能只修改局部变量
     * base，否则重新开中断后，TIM1 中断回调还会再次累加同一次
     * 溢出，导致时间戳突然增加 65536 us。
     */
    if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))
    {
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
        g_us += 0x10000ULL;
        base = g_us;
        cnt  = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1); // 重新读一次 CNT
    }

    __enable_irq();

    return base + cnt;
}
