#ifndef DRV8701_H
#define DRV8701_H

/* DRV8701E 双路电机控制接口，函数名与原 AT8236 驱动保持一致。 */

#include <stdint.h>
#include "main.h"

/* 初始化两片 DRV8701E 使用的 PH/EN PWM 输出。 */
void Motor_Init(void);

/* 使能或禁止电机输出；传入 0 时立即刹车停止两侧电机。 */
void Motor_Cmd(uint8_t on);

/* 设置左、右电机占空比，范围为 -100.0f 至 +100.0f。 */
void Motor_Set_L(float duty);
void Motor_Set_R(float duty);

/* 停止单侧或两侧电机。 */
void Motor_Stop_L(void);
void Motor_Stop_R(void);
void Motor_Stop(void);

#endif /* DRV8701_H */
