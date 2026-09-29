#include "../Inc/app_pwm.h"
#include <math.h>
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

//
// @简介：对TB6612进行初始化
//
void App_PWM_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

//
// @简介：控制TB6612进入休眠状态或者活动状态
// @参数：on    0 - 休眠状态，向STBY写L
//           非零 - 活动状态，向STBY写H
//
void App_PWM_Cmd(uint8_t on)
{
    if(on == 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // 休眠
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // 休眠
    }
}
//
//@brief：设置左电机占空比
//@pram:duty - 占空比的具体值，范围-100.0f - +100.0f
//
void App_PWM_Set_L(float duty) {
    float sign;//符号，占空比为正sign为+1，占空比为负sign为-1

    if (duty >= 0) {
        sign = 1;
    }else {
        sign = -1;
    }//判断是否正负之后即可取绝对值来判断转速
    //duty = duty * sign;//这也是个好方法，如果是负数乘负一的话就变成正数了，不过这里我还是要用绝对值

    duty = fabsf(duty);//对浮点数duty取绝对值

    //设置正反转
    if (sign < 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    }else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,  GPIO_PIN_RESET);  // AIN1 - 低
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    }
    //设置转速（设置占空比）
    uint16_t ccr = duty / 100.0f * 999;//注意两个整数作除法一定要用.0f或者前面乘个1.0
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,ccr);
}
//
//@brief：设置左电机占空比
//@pram:duty - 占空比的具体值，范围-100.0f - +100.0f
//
void App_PWM_Set_R(float duty) {
    float sign;//符号，占空比为正sign为+1，占空比为负sign为-1

    if (duty >= 0) {
        sign = 1;
    }else {
        sign = -1;
    }//判断是否正负之后即可取绝对值来判断转速
    //duty = duty * sign;//这也是个好方法，如果是负数乘负一的话就变成正数了，不过这里我还是要用绝对值

    duty = fabsf(duty);//对浮点数duty取绝对值

    //设置正反转
    if (sign > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    }else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,  GPIO_PIN_RESET);  // BIN1 - 低
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    }
    //设置转速（设置占空比）
    uint16_t ccr = duty / 100.0f * 999;//注意两个整数作除法一定要用.0f或者前面乘个1.0
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,ccr);
}

