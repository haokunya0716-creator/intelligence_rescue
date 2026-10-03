/*
 * DRV8701E PH/EN 电机驱动。
 *
 * 当前硬件连接为：
 *   左电机：PD12(TIM4_CH1) = EN_1，PD9 = PH_1_IN
 *   右电机：PD13(TIM4_CH2) = EN_2，PD10 = PH_2_IN
 *
 * 每侧需要一片 DRV8701E。PH 通道使用普通 GPIO 输出静态逻辑电平，
 * EN 通道输出速度 PWM。DRV8701E 的 nSLEEP 必须由硬件
 * 拉高（或在工程中另行配置 GPIO 控制），nFAULT 为开漏故障输出。
 */

#include "drv8701.h"
#include "tim.h"

#define MOTOR_TIMER_HANDLE       (&htim4)
#define MOTOR_LEFT_EN_CHANNEL    TIM_CHANNEL_1
#define MOTOR_RIGHT_EN_CHANNEL   TIM_CHANNEL_2

static uint8_t motor_enabled = 0U;

static float Motor_LimitDuty(float duty)
{
    if (duty > 100.0f)
    {
        duty = 100.0f;
    }
    else if (duty < -100.0f)
    {
        duty = -100.0f;
    }

    return duty;
}

/* 返回 PWM 比较寄存器可用的最大值（ARR）。 */
static uint32_t Motor_GetPwmMax(void)
{
    return __HAL_TIM_GET_AUTORELOAD(MOTOR_TIMER_HANDLE);
}

/* 将占空比绝对值转换为 PWM1 的高电平比较值。 */
static uint32_t Motor_DutyToCompare(float duty)
{
    uint32_t pwm_max = Motor_GetPwmMax();

    if (duty < 0.0f)
    {
        duty = -duty;
    }
    if (duty > 100.0f)
    {
        duty = 100.0f;
    }

    return (uint32_t)(duty * (float)pwm_max / 100.0f + 0.5f);
}

static void Motor_SetChannel(uint32_t channel, uint32_t compare)
{
    uint32_t pwm_max = Motor_GetPwmMax();

    if (compare > pwm_max)
    {
        compare = pwm_max;
    }
    __HAL_TIM_SET_COMPARE(MOTOR_TIMER_HANDLE, channel, compare);
}

/* PH=1 为正向，PH=0 为反向；EN 由调用者设置速度 PWM。 */
static void Motor_SetPhase(GPIO_TypeDef *port, uint16_t pin, uint8_t high)
{
    HAL_GPIO_WritePin(port, pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor_Stop_L(void)
{
    /* EN=0：DRV8701E 进入低侧慢衰减刹车状态。 */
    Motor_SetChannel(MOTOR_LEFT_EN_CHANNEL, 0U);
}

void Motor_Stop_R(void)
{
    Motor_SetChannel(MOTOR_RIGHT_EN_CHANNEL, 0U);
}

void Motor_Stop(void)
{
    Motor_Stop_L();
    Motor_Stop_R();
}

void Motor_Init(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    motor_enabled = 0U;
    Motor_Stop();
    Motor_SetPhase(PH_1_IN_GPIO_Port, PH_1_IN_Pin, 0U);
    Motor_SetPhase(PH_2_IN_GPIO_Port, PH_2_IN_Pin, 0U);

    if (HAL_TIM_PWM_Start(MOTOR_TIMER_HANDLE, MOTOR_LEFT_EN_CHANNEL) != HAL_OK)
    {
        status = HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(MOTOR_TIMER_HANDLE, MOTOR_RIGHT_EN_CHANNEL) != HAL_OK)
    {
        status = HAL_ERROR;
    }

    if (status != HAL_OK)
    {
        HAL_TIM_PWM_Stop(MOTOR_TIMER_HANDLE, MOTOR_LEFT_EN_CHANNEL);
        HAL_TIM_PWM_Stop(MOTOR_TIMER_HANDLE, MOTOR_RIGHT_EN_CHANNEL);
        Motor_Stop();
        return;
    }

    motor_enabled = 1U;
    Motor_Cmd(1U);
}

void Motor_Cmd(uint8_t on)
{
    motor_enabled = (on != 0U) ? 1U : 0U;

    if (motor_enabled == 0U)
    {
        Motor_Stop();
    }
}

void Motor_Set_L(float duty)
{
    if (motor_enabled == 0U)
    {
        Motor_Stop_L();
        return;
    }

    duty = Motor_LimitDuty(-duty);
    if (duty == 0.0f)
    {
        Motor_Stop_L();
        return;
    }

    Motor_SetPhase(PH_1_IN_GPIO_Port, PH_1_IN_Pin, (duty > 0.0f) ? 1U : 0U);
    Motor_SetChannel(MOTOR_LEFT_EN_CHANNEL, Motor_DutyToCompare(duty));
}

void Motor_Set_R(float duty)
{
    if (motor_enabled == 0U)
    {
        Motor_Stop_R();
        return;
    }

    duty = Motor_LimitDuty(duty);
    if (duty == 0.0f)
    {
        Motor_Stop_R();
        return;
    }

    Motor_SetPhase(PH_2_IN_GPIO_Port, PH_2_IN_Pin, (duty > 0.0f) ? 1U : 0U);
    Motor_SetChannel(MOTOR_RIGHT_EN_CHANNEL, Motor_DutyToCompare(duty));
}
