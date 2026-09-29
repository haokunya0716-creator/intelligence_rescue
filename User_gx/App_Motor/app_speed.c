#include "app_speed.h"
#include "app_encoder.h"
#include "at8236.h"
#include "pid.h"
#include "task.h"

static PID_TypeDef pid_speed_r; // 右电机速度闭环，输入和反馈单位都是 cm/s

volatile float speed_l_measure = 0.0f;
volatile float speed_r_measure = 0.0f; // 实际速度，单位 cm/s

volatile float speed_l_out = 0.0f;
volatile float speed_r_out = 0.0f; // 右轮总输出占空比

volatile float speed_l_ref = 0.0f;
volatile float speed_r_ref = 0.0f; // 目标速度

/*
 * 右轮速度环初始参数。
 *
 * 控制量单位为 PWM 占空比百分数，速度单位为 cm/s：
 * duty = speed_dependent_feedforward(target)
 *      + KP_R * (target - feedback)
 *
 * Ki 和 Kd 固定为 0，避免积分累积和测速噪声引入微分尖峰。
 */
#define KP_R                    0.50f
#define KI_R                    0.00f
#define KD_R                    0.00f
#define KF_R                    1.10f
#define FF_STATIC_R             7.50f
#define FF_OFFSET_CORRECTION_R -0.72f
#define FF_LINEAR_CORRECTION_R  0.0385f
#define FF_QUADRATIC_CORRECTION_R 0.00152f
#define SPEED_OUTPUT_LIMIT_R    80.0f

static uint8_t speed_filter_init = 0U;
static float speed_r_filter = 0.0f;
static uint32_t speed_filter_time = 0U;

static void App_Speed_FilterReset(void)
{
    speed_filter_init = 0U;
    speed_r_filter = 0.0f;
    speed_filter_time = 0U;
}

static float App_Speed_Limit(float value, float upper, float lower)
{
    if(value > upper){
        return upper;
    }

    if(value < lower){
        return lower;
    }

    return value;
}

static float App_Speed_FeedForward(float speed_ref)
{
    float speed_abs;
    float friction_compensation;

    speed_abs = speed_ref >= 0.0f ? speed_ref : -speed_ref;
    friction_compensation = FF_STATIC_R
                            + FF_OFFSET_CORRECTION_R
                            + FF_LINEAR_CORRECTION_R * speed_abs
                            + FF_QUADRATIC_CORRECTION_R * speed_abs * speed_abs;

    if(speed_ref > 0.0f){
        return KF_R * speed_ref + friction_compensation;
    }

    if(speed_ref < 0.0f){
        return KF_R * speed_ref - friction_compensation;
    }

    return 0.0f;
}

void App_Motor_Data_Update(void)
{
    float speed_r_raw = App_Encoder_GetLinearSpeed_R();
    uint32_t now_ms = HAL_GetTick();
    float dt = 0.0f;
    float alpha = 0.0f;

    if(speed_filter_init == 0U){
        speed_r_filter = speed_r_raw;
        speed_filter_time = now_ms;
        speed_filter_init = 1U;
    }else {
        dt = (now_ms - speed_filter_time) * 0.001f;
        speed_filter_time = now_ms;

        if(dt > 0.0f){
            alpha = dt / (0.03f + dt);
            speed_r_filter += alpha * (speed_r_raw - speed_r_filter);
        }
    }

    speed_l_measure = 0.0f;
    speed_r_measure = speed_r_filter;
    PID_ChangeSP(&pid_speed_r, speed_r_ref);
}

void App_Speed_Init(void)
{
    PID_Init(&pid_speed_r, KP_R, KI_R, KD_R);
    PID_LimitConfig(&pid_speed_r,
                    SPEED_OUTPUT_LIMIT_R,
                    -SPEED_OUTPUT_LIMIT_R);

    App_Speed_FilterReset();
    speed_l_measure = 0.0f;
    speed_r_measure = 0.0f;
    speed_l_out = 0.0f;
    speed_r_out = 0.0f;
    speed_l_ref = 0.0f;
    speed_r_ref = 0.0f;
}

//
// @简介：设置左右电机目标速度。
// @说明：
// 1. speed_l/speed_r 的单位是 cm/s。
// 2. 当前调参模式只接受右轮目标，左轮目标强制为 0。
//
void App_Speed_Set(float speed_l, float speed_r)
{
    (void)speed_l;

    speed_l_ref = 0.0f;
    speed_r_ref = speed_r;

    if(speed_r_ref == 0.0f){
        App_Speed_FilterReset();
        speed_l_measure = 0.0f;
        speed_r_measure = 0.0f;
        speed_l_out = 0.0f;
        speed_r_out = 0.0f;
        Motor_Stop();
        App_Speed_Reset();
    }
}

//
// @简介：复位右轮速度环的历史状态。
//
void App_Speed_Reset(void)
{
    PID_Reset(&pid_speed_r);
}

//
// @简介：右轮 P + 前馈速度闭环。
// @说明：
// 1. 内部按 5 ms 周期运行。
// 2. 只读取、计算和驱动右轮。
// 3. 左轮每个周期保持停止。
//
void App_Speed_Pro(void)
{
    static uint8_t right_control_active = 0U;

    App_Motor_Data_Update();

    PERIODIC_START(SPEED_PID, 5)

    /* The left wheel is deliberately disabled for this tuning run. */
    Motor_Stop_L();
    speed_l_out = 0.0f;
    speed_l_ref = 0.0f;

    if(speed_r_ref == 0.0f){
        speed_r_measure = 0.0f;
        speed_r_out = 0.0f;
        Motor_Stop_R();

        if(right_control_active != 0U){
            App_Speed_FilterReset();
            App_Speed_Reset();
            right_control_active = 0U;
        }
    }else {
        speed_r_out = PID_Compute(&pid_speed_r, speed_r_measure);
        speed_r_out += App_Speed_FeedForward(speed_r_ref);
        speed_r_out = App_Speed_Limit(speed_r_out,
                                      SPEED_OUTPUT_LIMIT_R,
                                      -SPEED_OUTPUT_LIMIT_R);
        Motor_Set_R(speed_r_out);
        right_control_active = 1U;
    }

    PERIODIC_END
}

//
// @简介：运行右轮自动阶跃测试。
// @说明：上电后等待 1 秒，依次运行 5、10、15、20、30、36 cm/s，
//        每级保持 4 秒、停止 2 秒，测试结束后自动停止。
//
void App_Speed_AutoTest_Pro(void)
{
    static const float test_targets[] = {
        5.0f, 10.0f, 15.0f, 20.0f, 30.0f, 36.0f
    };
    static uint32_t test_start_ms = 0U;
    static uint8_t test_initialized = 0U;
    uint32_t elapsed_ms;
    uint32_t profile_ms;
    uint32_t stage_ms;
    uint8_t stage;

    if(test_initialized == 0U){
        test_start_ms = HAL_GetTick();
        test_initialized = 1U;
        App_Speed_Set(0.0f, 0.0f);
        return;
    }

    elapsed_ms = HAL_GetTick() - test_start_ms;

    if(elapsed_ms < 1000U){
        App_Speed_Set(0.0f, 0.0f);
        return;
    }

    profile_ms = elapsed_ms - 1000U;
    stage = (uint8_t)(profile_ms / 6000U);
    if(stage >= (uint8_t)(sizeof(test_targets) / sizeof(test_targets[0]))){
        App_Speed_Set(0.0f, 0.0f);
        test_start_ms = HAL_GetTick();
        return;
    }

    stage_ms = profile_ms % 6000U;
    if(stage_ms < 4000U){
        App_Speed_Set(0.0f, test_targets[stage]);
    }else {
        App_Speed_Set(0.0f, 0.0f);
    }
}
