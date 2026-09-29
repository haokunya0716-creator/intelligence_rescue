#ifndef APP_PWM_H
#define APP_PWM_H

#include "main.h"

void App_PWM_Init(void);
void App_PWM_Cmd(uint8_t on);//唤醒电机专用
void App_PWM_Set_L(float duty);//设置左电机
void App_PWM_Set_R(float duty);//设置右电机

#endif
