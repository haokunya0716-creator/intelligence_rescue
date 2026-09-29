//
// Created by gaoxu on 2026/9/19.
//

#include "at8236.h"
#include "tim.h"

/*
 * AT8236 PWM 配置说明
 * --------------------
 * 原 TI 代码使用 TI DriverLib 的定时器实例和通道索引：
 *
 * 同款 STM32 工程采用的实际通道映射为：
 *
 *     BIN2 -> TIM1_CH1
 *     BIN1 -> TIM1_CH2
 *     AIN2 -> TIM1_CH3
 *     AIN1 -> TIM1_CH4
 *
 * STM32 HAL 使用定时器句柄和 TIM_CHANNEL_x 表示同样的资源。
 * 下面的句柄和通道宏是本文件与具体硬件之间的唯一连接点：
 * 如果 CubeMX 最终把 AT8236 配置到了其他定时器，只需要修改这些
 * 宏，不需要改动下面的电机控制逻辑。
 *
 * 当前先按同款 STM32 工程使用 htim1。对应的 GPIO 复用引脚应在
 * 注意：当前工程的 htim1 仍是原有基础定时器配置，只有在 CubeMX
 * 中将 TIM1 改为四路 PWM 并配置上述 GPIO 复用后，HAL_TIM_PWM_Start()
 * 才会真正产生 AT8236 所需的 PWM 波形。
 */
#define MOTOR_TIMER_HANDLE    (&htim2)
#define MOTOR_BIN2_CHANNEL    TIM_CHANNEL_1
#define MOTOR_BIN1_CHANNEL    TIM_CHANNEL_2
#define MOTOR_AIN2_CHANNEL    TIM_CHANNEL_3
#define MOTOR_AIN1_CHANNEL    TIM_CHANNEL_4

/*
 * PWM 周期不再写死为某个常量，而是直接读取 STM32 定时器的 ARR。
 * 这样无论 CubeMX 将 PWM 周期设置为 199、999 还是其他值，占空比
 * 换算都能自动与实际定时器配置保持一致。
 */
/* 电机输出使能标志：1 表示允许设置 PWM，0 表示输出处于停止状态。 */
static uint8_t motor_enabled = 0U;

/**
 * @brief 将电机占空比限制在合法范围内。
 *
 * AT8236 驱动接口使用浮点数表示占空比，其中正负号表示旋转方向，
 * 绝对值表示速度大小。为了避免调用者传入超过物理范围的数值，
 * 本函数会把所有大于 +100.0 的值限制为 +100.0，把所有小于
 * -100.0 的值限制为 -100.0。
 *
 * @param duty 需要限制的原始占空比，单位为百分比，允许范围为
 *             -100.0f 至 +100.0f。
 *
 * @return float 限制后的占空比。
 */
static float Motor_LimitDuty(float duty)
{
    if (duty > 100.0f)
    {
        duty = 100.0f;
    }

    if (duty < -100.0f)
    {
        duty = -100.0f;
    }

    return duty;
}

/**
 * @brief 获取当前 PWM 定时器的一个完整周期对应的计数数量。
 *
 * STM32 定时器的自动重装载寄存器 ARR 表示计数上限，定时器通常
 * 依次计数 0、1、...、ARR，因此一个完整周期包含 ARR + 1 个计数。
 * 使用实际 ARR 而不是固定常量，可以避免修改 CubeMX 的 PWM 周期
 * 后，电机占空比换算仍使用旧周期的问题。
 *
 * @return uint32_t 当前定时器 PWM 周期的计数数量。
 *
 * @note 当 ARR 已经达到 uint32_t 最大值时，ARR + 1 会发生整数溢出。
 *       这种配置无法用 uint32_t 表示“ARR + 1”，此时返回 ARR 本身，
 *       可避免溢出导致比较值异常。
 */
static uint32_t Motor_GetPwmPeriodCount(void)
{
    uint32_t autoreload = __HAL_TIM_GET_AUTORELOAD(MOTOR_TIMER_HANDLE);

    if (autoreload < UINT32_MAX)
    {
        return autoreload + 1U;
    }

    return autoreload;
}

/**
 * @brief 将占空比转换为定时器比较值。
 *
 * 原 TI 程序使用的是“周期计数值减去有效计数值”的比较值计算方式，
 * 因此这里保留完全相同的换算关系，以保证移植后电机控制逻辑不变。
 * 本函数只处理占空比的绝对值，正负方向由 Motor_Set_L() 和
 * Motor_Set_R() 分别通过选择不同的 PWM 通道实现。
 *
 * @param duty 占空比绝对值，单位为百分比。函数内部也会再次处理
 *             负值和超过 100.0 的异常值。
 *
 * @return uint32_t STM32 定时器捕获比较寄存器应写入的比较值。
 */
static uint32_t Motor_DutyToCompare(float duty)
{
    uint32_t period_count = Motor_GetPwmPeriodCount();//得到总周期
    uint32_t active_count = 0U;

    if (duty < 0.0f)
    {
        duty = -duty;
    }

    if (duty > 100.0f)
    {
        duty = 100.0f;
    }

    /*
     * 加 0.5f 后转换为整数，可以实现四舍五入，减少浮点数截断
     * 对低占空比控制造成的误差。
     */
    active_count = (uint32_t)(duty / 100.0f *
                              (float)period_count + 0.5f);

    if (active_count > period_count)
    {
        active_count = period_count;
    }

    return period_count - active_count;
}

/**
 * @brief 设置 AT8236 的某一路 PWM 比较值。
 *
 * 该函数负责把比较值写入 STM32 定时器的对应捕获比较寄存器，
 * 相当于原 TI 平台中的“设置定时器比较值”操作。比较值会先
 * 进行上限保护，防止错误参数超出当前 PWM 周期。
 *
 * @param channel STM32 HAL 定义的定时器通道，例如 TIM_CHANNEL_1。
 * @param compare 要写入捕获比较寄存器的比较值。
 *
 * @note MOTOR_TIMER_HANDLE 以及各路通道映射在本文件顶部统一定义，
 *       便于根据实际硬件接线调整。
 */
static void Motor_SetChannel(uint32_t channel, uint32_t compare)
{
    uint32_t period_count = Motor_GetPwmPeriodCount();

    if (compare > period_count)
    {
        compare = period_count;
    }

    __HAL_TIM_SET_COMPARE(MOTOR_TIMER_HANDLE, channel, compare);
}

/**
 * @brief 停止左侧电机。
 *
 * 左电机由 AIN1 和 AIN2 两个输入端组成。停止时将两路比较值都
 * 设置为 PWM 周期值，使两路输出回到原 TI 程序定义的停止状态。
 * 这里不关闭定时器，只改变输出比较值，以便后续能够快速恢复控制。
 */
void Motor_Stop_L(void)
{
    uint32_t period_count = Motor_GetPwmPeriodCount();

    Motor_SetChannel(MOTOR_AIN1_CHANNEL, period_count);
    Motor_SetChannel(MOTOR_AIN2_CHANNEL, period_count);
}

/**
 * @brief 停止右侧电机。
 *
 * 右电机由 BIN1 和 BIN2 两个输入端组成。停止时将两路比较值都
 * 设置为 PWM 周期值，保持与原 TI 版本的停止行为一致。
 */
void Motor_Stop_R(void)
{
    uint32_t period_count = Motor_GetPwmPeriodCount();

    Motor_SetChannel(MOTOR_BIN1_CHANNEL, period_count);
    Motor_SetChannel(MOTOR_BIN2_CHANNEL, period_count);
}

/**
 * @brief 初始化 AT8236 的四路 PWM 输出。
 *
 * 本函数完成以下工作：
 * 1. 启用当前配置的 PWM 定时器的四个通道；
 * 2. 先将左右电机设置为停止状态，避免初始化瞬间误转；
 * 3. 设置软件输出使能标志；
 * 4. 通过 Motor_Cmd() 统一确认输出状态。
 *
 * 原平台中启动 PWM 定时器的操作在 STM32 HAL 中由
 * HAL_TIM_PWM_Start() 完成。PWM 的周期、GPIO 复用功能和输出极性
 * 需要在 CubeMX 生成的定时器配置中完成。
 *
 * 本驱动保留原程序“周期计数值减去有效计数值”的比较值换算方式。
 * 因此，CubeMX 中的 PWM 模式和输出极性必须与实际 AT8236 输入端
 * 的有效电平保持一致；仅修改定时器句柄或通道编号，而不匹配输出
 * 极性，可能会导致占空比方向相反或停止状态不正确。
 */
void Motor_Init(void)
{
    HAL_StatusTypeDef pwm_status = HAL_OK;

    /*
     * 在启动定时器输出之前，先把四路比较寄存器预置为停止状态。
     * 这样即使 CubeMX 为 PWM 通道生成的初始 Pulse 不是安全值，调用
     * HAL_TIM_PWM_Start() 使能输出时也不会因为比较值尚未更新而产生
     * 一次意外的有效脉冲。
     */
    motor_enabled = 0U;
    Motor_Stop_L();
    Motor_Stop_R();

    /*
     * 四路 PWM 必须分别启动。即使四路使用同一个定时器，也需要
     * 针对每个通道调用一次 HAL_TIM_PWM_Start()。
     */
    if (HAL_TIM_PWM_Start(MOTOR_TIMER_HANDLE, MOTOR_AIN2_CHANNEL) != HAL_OK)
    {
        pwm_status = HAL_ERROR;
    }

    if (HAL_TIM_PWM_Start(MOTOR_TIMER_HANDLE, MOTOR_AIN1_CHANNEL) != HAL_OK)
    {
        pwm_status = HAL_ERROR;
    }

    if (HAL_TIM_PWM_Start(MOTOR_TIMER_HANDLE, MOTOR_BIN2_CHANNEL) != HAL_OK)
    {
        pwm_status = HAL_ERROR;
    }

    if (HAL_TIM_PWM_Start(MOTOR_TIMER_HANDLE, MOTOR_BIN1_CHANNEL) != HAL_OK)
    {
        pwm_status = HAL_ERROR;
    }

    if (pwm_status != HAL_OK)
    {
        /*
         * 任一路启动失败时，停止之前已经成功启动的通道，避免出现
         * “函数返回失败但部分 PWM 仍在运行”的不一致状态。停止调用
         * 即使某一路本身没有成功启动也不会影响其他通道。
         */
        HAL_TIM_PWM_Stop(MOTOR_TIMER_HANDLE, MOTOR_AIN2_CHANNEL);
        HAL_TIM_PWM_Stop(MOTOR_TIMER_HANDLE, MOTOR_AIN1_CHANNEL);
        HAL_TIM_PWM_Stop(MOTOR_TIMER_HANDLE, MOTOR_BIN2_CHANNEL);
        HAL_TIM_PWM_Stop(MOTOR_TIMER_HANDLE, MOTOR_BIN1_CHANNEL);

        /*
         * 失败时保持软件使能标志为 0，并再次写入停止值，确保调用者
         * 后续误调用 Motor_Set_L()/Motor_Set_R() 时不会驱动电机。
         */
        motor_enabled = 0U;
        Motor_Stop_L();
        Motor_Stop_R();
        return;
    }

    /*
     * 只有四路 PWM 都启动成功后，才允许后续速度控制函数改写
     * 比较值；否则保持禁用，避免调用者误以为电机已经正常工作。
     */
    motor_enabled = 1U;

    /* 通过统一接口将 AT8236 设置为允许输出状态。 */
    Motor_Cmd(1U);
}

/**
 * @brief 同时停止左右两侧电机。
 *
 * 该函数只修改四路 PWM 比较值，不关闭 PWM 定时器，也不改变
 * motor_enabled 标志。这样调用 Motor_Set_L()/Motor_Set_R() 时，
 * 如果输出仍处于使能状态，可以继续直接更新占空比。
 */
void Motor_Stop(void)
{
    Motor_Stop_L();
    Motor_Stop_R();
}

/**
 * @brief 使能或禁止 AT8236 的电机输出。
 *
 * @param on 0 表示禁止输出；非 0 表示允许 Motor_Set_L() 和
 *           Motor_Set_R() 更新 PWM。
 *
 * 禁止输出时会立即把左右电机设置为停止状态。重新使能时只恢复
 * 软件标志，不自动恢复上一次占空比；调用者需要再次设置目标速度。
 */
void Motor_Cmd(uint8_t on)
{
    motor_enabled = (on != 0U) ? 1U : 0U;

    if (motor_enabled == 0U)
    {
        Motor_Stop_L();
        Motor_Stop_R();
    }
}

/**
 * @brief 设置左侧电机的速度和旋转方向。
 *
 * @param duty 左侧电机占空比，范围为 -100.0f 至 +100.0f。
 *             其中正负号表示方向，绝对值表示速度大小。
 *
 * 函数内部保留了原 TI 版本中的 -duty 处理，因此对外传入值与
 * AIN1/AIN2 实际输出通道的对应关系，需要以原工程的机械方向定义为准：
 * - 经过 -duty 换算后为正时，AIN1 输出 PWM，AIN2 输出停止电平；
 * - 经过 -duty 换算后为负时，AIN2 输出 PWM，AIN1 输出停止电平；
 * - 传入 0 时，两路都设置为停止状态。
 *
 * 当电机输出被 Motor_Cmd(0) 禁止时，本函数不会输出 PWM，只会
 * 保持左侧电机停止。
 */
void Motor_Set_L(float duty)
{
    uint32_t period_count = Motor_GetPwmPeriodCount();
    uint32_t compare = 0U;

    if (motor_enabled == 0U)
    {
        Motor_Stop_L();
        return;
    }

    /*
     * 保留原 TI 代码中的 -duty 处理。该处理用于匹配原工程中左
     * 电机机械安装方向与右电机不同的情况。
     */
    duty = Motor_LimitDuty(-duty);
    compare = Motor_DutyToCompare(duty);

    if (duty > 0.0f)
    {
        /*
         * 经过前面的 -duty 换算后为正：
         * AIN1 为 PWM，AIN2 为停止电平。
         */
        Motor_SetChannel(MOTOR_AIN2_CHANNEL, period_count);
        Motor_SetChannel(MOTOR_AIN1_CHANNEL, compare);
    }
    else if (duty < 0.0f)
    {
        /*
         * 经过前面的 -duty 换算后为负：
         * AIN2 为 PWM，AIN1 为停止电平。
         */
        Motor_SetChannel(MOTOR_AIN1_CHANNEL, period_count);
        Motor_SetChannel(MOTOR_AIN2_CHANNEL, compare);
    }
    else
    {
        Motor_Stop_L();
    }
}

/**
 * @brief 设置右侧电机的速度和旋转方向。
 *
 * @param duty 右侧电机占空比，范围为 -100.0f 至 +100.0f。
 *             其中正负号表示方向，绝对值表示速度大小。
 *
 * 根据原 TI 版本的逻辑，右侧电机的输入方向为：
 * - 传入正值时，BIN1 输出 PWM，BIN2 输出停止电平；
 * - 传入负值时，BIN2 输出 PWM，BIN1 输出停止电平；
 * - 传入 0 时，两路都设置为停止状态。
 *
 * 当电机输出被 Motor_Cmd(0) 禁止时，本函数不会输出 PWM，只会
 * 保持右侧电机停止。
 */
void Motor_Set_R(float duty)
{
    uint32_t period_count = Motor_GetPwmPeriodCount();
    uint32_t compare = 0U;

    if (motor_enabled == 0U)
    {
        Motor_Stop_R();
        return;
    }

    duty = Motor_LimitDuty(duty);
    compare = Motor_DutyToCompare(duty);

    if (duty > 0.0f)
    {
        /* 右电机正向：BIN2 为停止电平，BIN1 为 PWM。 */
        Motor_SetChannel(MOTOR_BIN2_CHANNEL, period_count);
        Motor_SetChannel(MOTOR_BIN1_CHANNEL, compare);
    }
    else if (duty < 0.0f)
    {
        /* 右电机反向：BIN1 为停止电平，BIN2 为 PWM。 */
        Motor_SetChannel(MOTOR_BIN1_CHANNEL, period_count);
        Motor_SetChannel(MOTOR_BIN2_CHANNEL, compare);
    }
    else
    {
        Motor_Stop_R();
    }
}
