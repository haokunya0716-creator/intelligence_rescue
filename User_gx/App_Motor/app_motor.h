#ifndef APP_MOTOR_H_
#define APP_MOTOR_H_

#include <stdint.h>
#include "main.h"
#include "pid.h"

void App_Motor_Init(void);
void App_Update_Data(void);
void App_PID_Reset(void);
void App_Position_Angle_Reset(void);

/* angle_ref 为相对当前角度的变化量，正方向按 angle_yaw 的定义。 */
void Set_Angle_SP(float angle_ref);
void Set_Position_SP(float position_ref);
void Set_Position_Angle_SP(float position_ref, float angle_ref);

void App_Angle_Pro(void);
void App_Position_Pro(void);

extern float motor_left_speed;        // 左电机速度环目标值，单位：cm/s
extern float motor_right_speed;       // 右电机速度环目标值，单位：cm/s

extern int16_t motor_encoder1;        // 左电机编码器累计值，用来观察原始编码器计数
extern int16_t motor_encoder2;        // 右电机编码器累计值，用来观察原始编码器计数

extern float position_l;              // 左轮累计行驶距离，单位：cm
extern float position_r;              // 右轮累计行驶距离，单位：cm
extern float position_mid;            // 左右轮平均行驶距离，位置环反馈值，单位：cm
extern float position_mid_ref;        // 位置环目标距离，单位：cm

extern float angle_yaw;               // 当前偏航角，单位：度，顺时针为正
extern float angle_yaw_ref;           // 角度环目标偏航角，单位：度，顺时针为正

extern float motor_base_speed;        // 位置环输出的基础速度，单位：cm/s
extern float motor_turn_speed;        // 角度环输出的转向修正速度，单位：cm/s

extern volatile uint8_t motor_pos_done;       // 位置环完成标志，1：完成，0：未完成
extern volatile uint8_t motor_angle_done;     // 角度环完成标志，1：完成，0：未完成

#endif
