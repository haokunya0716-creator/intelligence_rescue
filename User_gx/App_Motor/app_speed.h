#ifndef APP_SPEED_H_
#define APP_SPEED_H_

#include "main.h"

void App_Motor_Data_Update(void);
//
// @简介：初始化左右电机速度闭环 PID。
//
void App_Speed_Init(void);

//
// @简介：设置左右电机目标速度。
// @参数：speed_l 左电机目标速度，单位 cm/s，正数表示当前定义的正转方向。
// @参数：speed_r 右电机目标速度，单位 cm/s，正数表示当前定义的正转方向。
//
void App_Speed_Set(float speed_l, float speed_r);



//
// @简介：复位速度 PID 的历史误差和积分项。
//
void App_Speed_Reset(void);


//
// @简介：速度闭环周期处理函数，需要在 while(1) 中持续调用。
//
void App_Speed_Pro(void);

extern volatile float speed_l_measure;//测出的实际速度
extern volatile float speed_r_measure;
extern volatile float speed_l_out;//输出占空比
extern volatile float speed_r_out;
extern volatile float speed_l_ref;
extern volatile float speed_r_ref;//目标速度

#endif
