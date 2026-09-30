#include "app_speed.h"
#include "app_encoder.h"
#include "at8236.h"
#include "pid.h"
#include "task.h"

static PID_TypeDef pid_speed_l; // 左电机速度闭环 PID，输入和反馈单位都是 cm/s
static PID_TypeDef pid_speed_r; // 右电机速度闭环 PID，输出为电机占空比

volatile float speed_l_measure = 0.0f;//实际速度，单位cm/s
volatile float speed_r_measure = 0.0f;

volatile float speed_l_out = 0.0f;//输出占空比
volatile float speed_r_out = 0.0f;

volatile float speed_l_ref = 0.0f;
volatile float speed_r_ref = 0.0f;//目标速度



static float KF_L = 1.2;  //动态前馈系数
static float KF_R = 1.3;  //动态前馈系数

//
//@brief：更新数据
//
static uint8_t speed_filter_init = 0;
static float speed_l_filter = 0.0f;
static float speed_r_filter = 0.0f;
static unsigned long speed_filter_time = 0;

static void App_Speed_FilterReset(void)
{
    speed_filter_init = 0;
    speed_l_filter = 0.0f;
    speed_r_filter = 0.0f;
    speed_filter_time = 0;
}

void App_Motor_Data_Update(void){

	float speed_l_raw = App_Encoder_GetLinearSpeed_L();
	float speed_r_raw = App_Encoder_GetLinearSpeed_R();
	uint32_t now_ms = HAL_GetTick();
	float dt = 0.0f;
	float alpha = 0.0f;

	if(speed_filter_init == 0){
		speed_l_filter = speed_l_raw;
		speed_r_filter = speed_r_raw;
		speed_filter_time = now_ms;
		speed_filter_init = 1;
	}else {
		dt = (now_ms - speed_filter_time) * 0.001f;
		speed_filter_time = now_ms;

		if(dt > 0.0f){
			alpha = dt / (0.03f + dt);
			speed_l_filter += alpha * (speed_l_raw - speed_l_filter);
			speed_r_filter += alpha * (speed_r_raw - speed_r_filter);
		}
	}

	speed_l_measure = speed_l_filter;
	speed_r_measure = speed_r_filter;

	PID_ChangeSP(&pid_speed_l,speed_l_ref);
	PID_ChangeSP(&pid_speed_r,speed_r_ref);//设定目标速度

}



void App_Speed_Init(void)
{
    PID_Init(&pid_speed_l,2.7f, 0.0f, 0.0f);
    PID_LimitConfig(&pid_speed_l, 70.0f, -70.0f);

    PID_Init(&pid_speed_r, 2.7f, 0.0f, 0.0f);
    PID_LimitConfig(&pid_speed_r, 70.0f, -70.0f);


}

//
// @简介：设置左右电机目标速度。
// @说明：
// 1. speed_l/speed_r 的单位是 cm/s。
// 2. 非零目标只改变设定值；目标为 0 时会立即把对应电机占空比清零。
//
void App_Speed_Set(float speed_l, float speed_r)
{
		speed_l_ref = speed_l;
		speed_r_ref = speed_r;
	  if(speed_l_ref == 0 && speed_r_ref == 0){

			App_Speed_FilterReset();
			speed_l_measure = 0.0f;
			speed_r_measure = 0.0f;
			speed_l_out = 0.0f;
			speed_r_out = 0.0f;
			Motor_Stop();//如果速度都为0，直接急停
			App_Speed_Reset();//同时复位pid

		}

}


//
// @简介：复位速度 PID。
// @说明：只清历史误差和积分项，不改变当前目标速度。
//
void App_Speed_Reset(void)
{
   PID_Reset(&pid_speed_l);
	 PID_Reset(&pid_speed_r);
}



//
// @简介：速度闭环周期处理。
// @说明：
// 1. 需要放在 while(1) 中一直调用，内部按 tick_ms 控制 20ms 运行一次。
// 2. 每次运行先读编码器速度，再用 PID 算占空比，最后输出给 AT8236。
// 3. Motor_Set_L() 内部已经处理左电机硬件方向反向，这里不再额外改符号。
//
void App_Speed_Pro(void)
{

	App_Motor_Data_Update();


	PERIODIC_START(SPEED_PID,15)

	if(speed_l_ref == 0 && speed_r_ref == 0){

		App_Speed_FilterReset();
		speed_l_measure = 0.0f;
		speed_r_measure = 0.0f;
		speed_l_out = 0.0f;
		speed_r_out = 0.0f;
		Motor_Stop();//如果速度都为0，直接急停
		App_Speed_Reset();//同时复位pid
	}else {

		speed_l_out = PID_Compute(&pid_speed_l,speed_l_measure);
		speed_r_out = PID_Compute(&pid_speed_r,speed_r_measure);

		Motor_Set_L(speed_l_out + KF_L * speed_l_ref);
		Motor_Set_R(speed_r_out + KF_R * speed_r_ref);

	}



	PERIODIC_END

}
