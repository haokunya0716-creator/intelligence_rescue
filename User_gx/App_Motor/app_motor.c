#include "app_motor.h"
#include "task.h"
#include "app_encoder.h"
#include "JY901S.h"
#include <math.h>

#include "app_speed.h"

static PID_TypeDef pid_position_mid;
static PID_TypeDef pid_angle_yaw;

float motor_left_speed = 0.0f;
float motor_right_speed = 0.0f;

int16_t motor_encoder1 = 0;
int16_t motor_encoder2 = 0;

float position_l = 0.0f;
float position_r = 0.0f;
float position_mid = 0.0f;
float position_mid_ref = 0.0f;

float angle_yaw = 0.0f;
float angle_yaw_ref = 0.0f;

float motor_base_speed = 0.0f;
float motor_turn_speed = 0.0f;

volatile uint8_t motor_pos_done = 1;
volatile uint8_t motor_angle_done = 1;

static float App_Motor_Abs(float x)
{
    if(x < 0.0f){
        return -x;
    }

    return x;
}

/*
 * 将角度统一限制到 [-180°, 180°) 范围。
 *
 * JY901S 的角度在跨过 180° 时会从接近 +180° 变成接近 -180°。
 * 这两个数值表示的是同一个方向，不能直接把它们相减，否则会
 * 得到接近 360° 的错误角度误差。
 */
static float App_Motor_NormalizeAngle(float angle)
{
    while(angle >= 180.0f){
        angle -= 360.0f;
    }

    while(angle < -180.0f){
        angle += 360.0f;
    }

    return angle;
}

static float App_Motor_AngleError(float target, float current)
{
    float err = target - current;

    /*
     * 角度误差也必须归一化，这样小车始终沿最短方向转动。
     * 例如目标 179°、当前 -179° 时，误差应为 -2°，而不是 358°。
     */
    return App_Motor_NormalizeAngle(err);
}

static float App_Motor_PositionSpeed(float err)
{
    float speed = 0.0f;
    float err_abs = App_Motor_Abs(err);

    // 距离较远时匀速走，快到目标时再按剩余距离减速。
    if(err_abs > 18.0f){
        speed = 40.0f;
    }else {
        speed = 40.0f * err_abs / 18.0f;

        if(speed < 20.0f){
            speed = 20.0f;
        }
    }

    if(err < 0.0f){
        speed = -speed;
    }

    return speed;
}

static float App_Motor_PIDComputeAngle(PID_TypeDef *PID, float FB)
{
    float err = App_Motor_AngleError(PID->SP, FB);
    uint32_t now_ms = HAL_GetTick();
    uint64_t t_k = 0;
    uint32_t delta_ms = 0;
    float deltaT = 0.0f;
    float err_dev = 0.0f;
    float err_int = 0.0f;
    float COp = 0.0f;
    float COi = 0.0f;
    float COd = 0.0f;
    float CO = 0.0f;

    t_k = now_ms;
    delta_ms = now_ms - (unsigned long)PID->t_k_1;
    deltaT = delta_ms * 1.0e-3f;

    if(deltaT <= 0.0f){
        deltaT = 1.0e-3f;
    }

    if(PID->t_k_1 != 0){
        //积分分离
        /*
         * 误差本身是环形角度。若误差从 179° 变成 -179°，
         * 实际只变化了 2°，不能直接相减得到 -358°，否则
         * 微分项会瞬间产生很大的错误输出。
         */
        float err_delta = App_Motor_AngleError(err, PID->err_k_1);
        err_dev = err_delta / deltaT;

        if(fabsf(err) < 10.0f){
            err_int = PID->err_int_k_1 + (err + PID->err_k_1) * deltaT * 0.5f;
        }else {
            err_int = 0.0f;
        }
    }

    COp = PID->Kp * err;
    COi = PID->Ki * err_int;
    COd = PID->Kd * err_dev;
    CO = COp + COi + COd;

    PID->t_k_1 = t_k;
    PID->err_k_1 = err;
    PID->err_int_k_1 = err_int;

    if(CO > PID->UpperLimit){
        CO = PID->UpperLimit;
    }

    if(CO < PID->LowerLimit){
        CO = PID->LowerLimit;
    }

    if(PID->err_int_k_1 > PID->UpperLimit){
        PID->err_int_k_1 = PID->UpperLimit;
    }

    if(PID->err_int_k_1 < PID->LowerLimit){
        PID->err_int_k_1 = PID->LowerLimit;
    }

    return CO;
}

void App_Update_Data(void)
{
    /*
     * 读取 JY901S 的校准后偏航角，作为角度环反馈值。
     * IMU_GetAnglesCalibrated() 允许只读取 yaw，因此前两个参数
     * 传 NULL。再次归一化是为了保证进入 PID 的角度始终处于
     * [-180°, 180°) 范围。
     */
    IMU_GetAnglesCalibrated(NULL, NULL, &angle_yaw);
    angle_yaw = App_Motor_NormalizeAngle(angle_yaw);

    motor_encoder1 = App_Encoder_GetEncoder_L();
    motor_encoder2 = App_Encoder_GetEncoder_R();

    position_l = App_Encoder_GetPos_L() / 360.0f * PI * (WHEEL_DIAMETER / 10.0f);
    position_r = App_Encoder_GetPos_R() / 360.0f * PI * (WHEEL_DIAMETER / 10.0f);
    position_mid = (position_l + position_r) * 0.5f;
}

void App_Motor_Init(void)
{
    App_Update_Data();

    PID_Init(&pid_position_mid, 0.0f, 0.0f, 0.0f);
    PID_LimitConfig(&pid_position_mid, 45.0f, -45.0f);

    PID_Init(&pid_angle_yaw, 0.0f, 0.0f, 0.0f);
    /*
     * 当前角度 PID 参数仍保持原来的 0 值，不在本次角度边界
     * 修正中擅自改变。实际使用角度环前，需要根据小车响应调参。
     */
    PID_LimitConfig(&pid_angle_yaw, 50.0f, -50.0f);

    position_mid_ref = position_mid;
    angle_yaw_ref = App_Motor_NormalizeAngle(angle_yaw);
    PID_ChangeSP(&pid_position_mid, position_mid_ref);
    PID_ChangeSP(&pid_angle_yaw, angle_yaw_ref);

    motor_left_speed = 0.0f;
    motor_right_speed = 0.0f;
    motor_base_speed = 0.0f;
    motor_turn_speed = 0.0f;

    motor_pos_done = 1;
    motor_angle_done = 1;
}

void App_PID_Reset(void)
{
    PID_Reset(&pid_position_mid);
    PID_Reset(&pid_angle_yaw);
    App_Speed_Reset();
}

void App_Position_Angle_Reset(void)
{
    App_PID_Reset();

    motor_left_speed = 0.0f;
    motor_right_speed = 0.0f;
    motor_base_speed = 0.0f;
    motor_turn_speed = 0.0f;

    motor_pos_done = 1;
    motor_angle_done = 1;
}

void Set_Angle_SP(float angle_ref)
{
    App_Update_Data();

    /*
     * angle_ref 表示相对当前角度的目标变化量。
     * 例如当前为 170°，传入 30°，目标应为 -160°。
     */
    angle_yaw_ref = App_Motor_NormalizeAngle(angle_yaw + angle_ref);
    PID_ChangeSP(&pid_angle_yaw, angle_yaw_ref);
    PID_Reset(&pid_angle_yaw);

    motor_angle_done = 0;
}

void Set_Position_SP(float position_ref)
{
    App_Update_Data();

    position_mid_ref = position_mid + position_ref;
    angle_yaw_ref = App_Motor_NormalizeAngle(angle_yaw);
    PID_ChangeSP(&pid_position_mid, position_mid_ref);
    PID_ChangeSP(&pid_angle_yaw, angle_yaw_ref);
    PID_Reset(&pid_position_mid);
    PID_Reset(&pid_angle_yaw);

    motor_pos_done = 0;
    motor_angle_done = 0;
}

void Set_Position_Angle_SP(float position_ref, float angle_ref)
{
    App_Update_Data();

    position_mid_ref = position_mid + position_ref;
    /*
     * angle_ref 与 Set_Angle_SP() 保持相同含义，表示相对当前
     * 航向角的变化量，而不是直接使用的绝对航向角。
     */
    angle_yaw_ref = App_Motor_NormalizeAngle(angle_yaw + angle_ref);
    PID_ChangeSP(&pid_position_mid, position_mid_ref);
    PID_ChangeSP(&pid_angle_yaw, angle_yaw_ref);
    PID_Reset(&pid_position_mid);
    PID_Reset(&pid_angle_yaw);

    motor_pos_done = 0;
    motor_angle_done = 0;
}

void App_Angle_Pro(void)
{
    App_Update_Data();
    PERIODIC_START(ANGLE,30)

    if(App_Motor_Abs(App_Motor_AngleError(angle_yaw_ref, angle_yaw)) <= 0.7f){
        motor_angle_done = 1;
        motor_turn_speed = 0.0f;
        App_Speed_Set(0.0f, 0.0f);
        PID_Reset(&pid_angle_yaw);
    }else {
        motor_angle_done = 0;
        motor_turn_speed = App_Motor_PIDComputeAngle(&pid_angle_yaw, angle_yaw);

        if(motor_turn_speed > 0.0f && motor_turn_speed < 10.0f){
            motor_turn_speed = 5.0f;
        }else if(motor_turn_speed < 0.0f && motor_turn_speed > -10.0f){
            motor_turn_speed = -5.0f
        }

        App_Speed_Set(-motor_turn_speed, +motor_turn_speed);
    }
    PERIODIC_END

    App_Speed_Pro();
}

void App_Position_Pro(void)
{
    PERIODIC_START(POSITION,30)
    App_Update_Data();

    if(App_Motor_Abs(position_mid_ref - position_mid) <= 1.0f
       && App_Motor_Abs(App_Motor_AngleError(angle_yaw_ref, angle_yaw)) <= 0.7f){
        motor_pos_done = 1;
        motor_base_speed = 0.0f;
        motor_left_speed = 0.0f;
        motor_right_speed = 0.0f;
        motor_turn_speed = 0.0f;
        App_Speed_Set(0.0f, 0.0f);
        PID_Reset(&pid_position_mid);
        PID_Reset(&pid_angle_yaw);
    }else {
        motor_pos_done = 0;
        /*
         * 位置已经到达但角度还没有到达时，停止前进，只保留
         * 原地转向，避免位置误差很小时仍以最小速度向前移动。
         */
        if(App_Motor_Abs(position_mid_ref - position_mid) <= 1.0f){
            motor_base_speed = 0.0f;
        }else {
            motor_base_speed = App_Motor_PositionSpeed(position_mid_ref - position_mid);
        }

        if(App_Motor_Abs(App_Motor_AngleError(angle_yaw_ref, angle_yaw)) <= 0.7f){
            motor_angle_done = 1;
            motor_turn_speed = 0.0f;
        }else {
            motor_angle_done = 0;
            motor_turn_speed = App_Motor_PIDComputeAngle(&pid_angle_yaw, angle_yaw);
        }

        motor_left_speed = motor_base_speed - motor_turn_speed;
        motor_right_speed = motor_base_speed + motor_turn_speed;
        App_Speed_Set(motor_left_speed, motor_right_speed);
    }
    PERIODIC_END

    App_Speed_Pro();
}
