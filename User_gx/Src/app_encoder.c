//
// Created by gaoxu on 2026/2/7.
//

#include "../Inc/app_encoder.h"
#include "../Inc/gx_delay.h"

#define E1A_Pin GPIO_PIN_6
#define E1A_GPIO_Port GPIOA
#define E1B_Pin GPIO_PIN_7
#define E1B_GPIO_Port GPIOA

#define E2A_Pin GPIO_PIN_1
#define E2A_GPIO_Port GPIOB
#define E2B_Pin GPIO_PIN_0
#define E2B_GPIO_Port GPIOB

static volatile int64_t encoder_L = 0;//代表电机旋转角度，64位防止溢出，同时带符号（因为有正反转）
//加volatile是因为encoder是中断和常规程序共享变量
static volatile int64_t encoder_R = 0;
/////////////////////电机转向/////////////////////////////////////////////////////////////////////////////////////////
static volatile int8_t direction_l = 1;
static volatile int8_t direction_r = 1;
//代表电机转向，1为正转，-1为反转
//+2代表反转->正转,-2代表正转->反转
//////////////////////T法测速相关变量////////////////////////////////////////////////////////////////////////////////////////
static volatile uint64_t t0_l = 0;//左电机时间
static volatile uint64_t t1_l = 0;

static volatile uint64_t t0_r = 0;//右电机时间
static volatile uint64_t t1_r = 0;

int64_t App_Encoder_GetEncoder_L(void) {
   return encoder_L;
}
int64_t App_Encoder_GetEncoder_R(void){
    return encoder_R;
}


//减速比 30613.0f / 1500.0f ~= 20.4
//轮胎转一圈，磁圈转20.4圈（减速比），共有11个对极，所以磁圈转一圈encoder加(30613.0f / 1500.0f)*22.0f
//最终得到的是转了多少度（一圈是三百六十度）

float App_Encoder_GetPos_L(void) {
    return encoder_L / ENCODER_TOTAL_RESOLUTION * 360.0f;//记得带.0f
}

float App_Encoder_GetPos_R(void) {
    return encoder_R / ENCODER_TOTAL_RESOLUTION * 360.0f;
}
//
//@brief:读取左轮胎的角速度,单位 rad/s
//
float App_Encoder_GetSpeed_L(void) {

    __disable_irq();//关闭单片机总中断
    //把这些值拷贝下来（当成副本），发生中断的时候改变的是新一轮的值，而非副本值，这样就能做到副本值不变，也不影响原本值的中断处理
    int8_t direction_cpy = direction_l;
    uint64_t t0_l_cpy = t0_l;
    uint64_t t1_l_cpy = t1_l;

    __enable_irq();//打开中断
    if (direction_cpy == 2 || direction_cpy == -2 ||
        t0_l_cpy == 0U) {
        return 0.0f;
    }else {
        uint64_t now = gx_GetUs();
        uint64_t since_last_pulse;
        uint64_t pulse_period = 0U;
        uint64_t period_us;

        if (now <= t0_l_cpy) {
            return 0.0f;
        }

        since_last_pulse = now - t0_l_cpy;
        if (t1_l_cpy != 0U && t0_l_cpy > t1_l_cpy) {
            pulse_period = t0_l_cpy - t1_l_cpy;
        }

        if (since_last_pulse > ENCODER_SPEED_TIMEOUT_US) {
            return 0.0f;
        }

        period_us = pulse_period;
        if (period_us == 0U || period_us > since_last_pulse) {
            period_us = since_last_pulse;
        }

        if (period_us == 0U) {
            return 0.0f;
        }

        float T = period_us * 1.0e-6f;//注意t的单位是us
        return 1.0 * direction_cpy/ T / ENCODER_TOTAL_RESOLUTION  * 6.2831853f;
    }

}
//
//@brief:读取右轮胎的角速度
//
float App_Encoder_GetSpeed_R(void) {
    __disable_irq();//关闭单片机总中断
    //把这些值拷贝下来（当成副本），发生中断的时候改变的是新一轮的值，而非副本值，这样就能做到副本值不变，也不影响原本值的中断处理
    int8_t direction_cpy = direction_r;
    uint64_t t0_r_cpy = t0_r;
    uint64_t t1_r_cpy = t1_r;

    __enable_irq();//打开中断
    if (direction_cpy == 2 || direction_cpy == -2 ||
        t0_r_cpy == 0U) {
        return 0.0f;
    }else{
        uint64_t now = gx_GetUs();
        uint64_t since_last_pulse;
        uint64_t pulse_period = 0U;
        uint64_t period_us;

        if (now <= t0_r_cpy) {
            return 0.0f;
        }

        since_last_pulse = now - t0_r_cpy;
        if (t1_r_cpy != 0U && t0_r_cpy > t1_r_cpy) {
            pulse_period = t0_r_cpy - t1_r_cpy;
        }

        if (since_last_pulse > ENCODER_SPEED_TIMEOUT_US) {
            return 0.0f;
        }

        period_us = pulse_period;
        if (period_us == 0U || period_us > since_last_pulse) {
            period_us = since_last_pulse;
        }

        if (period_us == 0U) {
            return 0.0f;
        }

        float T = period_us * 1.0e-6f;//注意t的单位是us
        return 1.0 * direction_cpy/ T / ENCODER_TOTAL_RESOLUTION * 6.2831853f;
    }

}



/**
 * @brief 获取左轮当前的线速度
 * @return float 线速度，单位 cm/s
 */
float App_Encoder_GetLinearSpeed_L(void) {
    // 调用已有的度/秒函数，乘以转换系数
    return App_Encoder_GetSpeed_L() * WHEEL_DIAMETER * 0.05f;
}

/**
 * @brief 获取右轮当前的线速度
 * @return float 线速度，单位 cm/s
 */
float App_Encoder_GetLinearSpeed_R(void) {
    // 调用已有的度/秒函数，乘以转换系数
    return App_Encoder_GetSpeed_R() * WHEEL_DIAMETER * 0.05f;
}
//用引脚来捕捉AB相的上升沿，
//dire绝对值为2则说明现在处于换向
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    //左电机 ->
    if(GPIO_Pin == E1A_Pin) {
        t1_l = t0_l;//这次的t1是上次的t0
        t0_l = gx_GetUs();

        GPIO_PinState a_L = HAL_GPIO_ReadPin(E1A_GPIO_Port, E1A_Pin); // A相的当前电压
        GPIO_PinState b_L = HAL_GPIO_ReadPin(E1B_GPIO_Port, E1B_Pin); // B相的当前电压

        if ( (a_L == GPIO_PIN_SET && b_L == GPIO_PIN_RESET) || (a_L == GPIO_PIN_RESET && b_L == GPIO_PIN_SET) ) {

            encoder_L++;//正转

            //这里的direction是上一次的方向，这次还没更新
            if (direction_l < 0) {
                direction_l = +2;
            }
            else
            {
                direction_l = 1;
            }

        }else //轮胎反转
        {
            encoder_L--;

            if(direction_l > 0) // 之前轮胎是正转
            {
                direction_l = -2;
            }
            else
            {
                direction_l = -1;
            }

        }
        // //if上升沿else下降沿
        // if (a_L == GPIO_PIN_SET)
        // {
        //     if (b_L == GPIO_PIN_RESET)
        //     {
        //         encoder_L --;
        //         direction_l = -1;
        //     }else
        //     {
        //         encoder_L ++;
        //         direction_l = 1;
        //
        //     }
        // }else
        // {
        //     if (b_L == GPIO_PIN_SET)
        //     {
        //         encoder_L --;
        //         direction_l = -1;
        //
        //     }else
        //     {
        //         encoder_L ++;
        //         direction_l = 1;
        //
        //     }
        // }
    }
    //右电机
    if (GPIO_Pin == E2A_Pin) {
        uint64_t now_r = gx_GetUs();

        /*
         * 右轮 A 相在电机 PWM 期间会出现极短毛刺。
         * 最高调参目标为 36 cm/s，对应的有效编码器边沿周期
         * 明显大于 200 us，因此丢弃更短的间隔不会影响目标速度
         * 范围内的真实脉冲。
         */
        if (t0_r != 0U && now_r > t0_r &&
            now_r - t0_r < RIGHT_ENCODER_MIN_EDGE_INTERVAL_US) {
            return;
        }

        t1_r = t0_r;//这次的t1是上次的t0
        t0_r = now_r;
        GPIO_PinState a_R = HAL_GPIO_ReadPin(E2A_GPIO_Port, E2A_Pin); // A相的当前电压
        GPIO_PinState b_R = HAL_GPIO_ReadPin(E2B_GPIO_Port, E2B_Pin); // B相的当前电压
        if ( (a_R == GPIO_PIN_SET && b_R == GPIO_PIN_RESET) || (a_R == GPIO_PIN_RESET && b_R == GPIO_PIN_SET)) {
            encoder_R--;

            if(direction_r > 0) // 之前轮胎是正转
            {
                direction_r = -2;
            }
            else
            {
                direction_r = -1;
            }
        }else {
            encoder_R++;

            if(direction_r < 0) // 之前轮胎是反转，现在轮胎是正转
            {
                direction_r = +2;
            }
            else
            {
                direction_r = 1;
            }

        }
        // if (a_R == GPIO_PIN_SET) {
        //     if (b_R == GPIO_PIN_RESET) {
        //         encoder_R ++;
        //         direction_r = 1;
        //
        //     }else {
        //         encoder_R --;
        //         direction_r = -1;
        //     }
        // }else {
        //     if (b_R == GPIO_PIN_RESET) {
        //         encoder_R --;
        //         direction_r = -1;
        //     }else {
        //         encoder_R ++;
        //         direction_r = 1;
        //     }
        // }
    }
}
