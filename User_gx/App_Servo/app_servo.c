//
// Created by gaoxu on 2026/9/26.
//

//控制180读舵机
//此文件默认定时器频率为50Hz  --   20ms
//高电平脉冲的时间0.5ms的话，舵机转到0度，1ms -45度 1.5ms--90度 2ms--135度2.5ms-180度

//当前文件arr+1 = 10000

#include "app_servo.h"

#include "tim.h"

static uint32_t Servo_angle_Compute(float angle_target);
//
//@brief: 初始化函数
void App_Servo_init(void) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    App_Servo_SetAngle(0.0f);
}

void App_Servo_SetAngle(float angle) {
    uint32_t ccr = Servo_angle_Compute(angle);
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,ccr);
}

static uint32_t Servo_angle_Compute(float angle_target) {
    if (angle_target > 180.0f) {
        angle_target = 180.0f;
    }
    if (angle_target <= 0.0f) {
        angle_target = 0.0f;
    }
    // TIM3计数频率为500kHz，每个计数为2us
    // 500us对应250，2500us对应1250
    return 250U + (uint32_t)(angle_target * 1000.0f / 180.0f);
}
