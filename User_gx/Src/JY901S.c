
#include "../User_gx/Inc/JY901S.h"
#include <stddef.h> /* for NULL */
#include "stm32f4xx.h" /* for __disable_irq / __enable_irq (CMSIS) */
#include "Inc/app_usart.h"

/* 全局变量定义（本文件定义，需要在头文件中 extern 声明） */
volatile int16_t IMU_rawRoll = 0;
volatile int16_t IMU_rawPinch = 0;
volatile int16_t IMU_rawYaw = 0;

volatile float rollAngle = 0.0f;
volatile float pinchAngle = 0.0f;
volatile float yawAngle = 0.0f;

volatile uint8_t IMU_buffer[12];
volatile uint8_t IMU_bufferIdx = 0;

/* 角度零点偏移 */
static float roll_offset = 0.0f;
static float pinch_offset = 0.0f;
static float yaw_offset = 0.0f;

/* 角度零点校准（设备保持静止时调用） */
void IMU_CalibrateOffset(void)
{
    __disable_irq();

    roll_offset = rollAngle;
    pinch_offset = pinchAngle;
    yaw_offset = yawAngle;

    __enable_irq();
}



/* 内部：把缓冲区内容解析为原始值与角度（小端在前，两个字节一组） */
static void IMU_SaveBuffer(const uint8_t *buf)
{
    /* 先按小端拼成 uint16，再转换为 int16（有符号） */
    int16_t rawRoll = (int16_t)(((uint16_t)buf[2]) | ((uint16_t)buf[3] << 8));
    int16_t rawPinch = (int16_t)(((uint16_t)buf[4]) | ((uint16_t)buf[5] << 8));
    int16_t rawYaw = (int16_t)(((uint16_t)buf[6]) | ((uint16_t)buf[7] << 8));

    IMU_rawRoll = rawRoll;
    IMU_rawPinch = rawPinch;
    IMU_rawYaw = rawYaw;

    /* 映射为角度：raw / 32768.0f * 180.0f （与原实现保持一致） */
    /*角度范围是-180°——+180°*/
    rollAngle = (float)rawRoll / 32768.0f * 180.0f;
    pinchAngle = (float)rawPinch / 32768.0f * 180.0f;
    yawAngle = (float)rawYaw / 32768.0f * 180.0f;
}

/* 计算校验：对 buf[0..9]这10个字节 求和并返回低 8 位 */
uint8_t IMU_CheckSum(const uint8_t *buf)
{
    uint8_t i;
    uint8_t checkSum = 0x00; //定义为无符号8位整型
    if (buf == NULL) return 0;
    for (i = 0; i < 10; i++) {
        checkSum += buf[i];
    }
    return checkSum;
}

/* 逐字节接收的状态机：供串口 ISR / 回调使用 */
void IMU_PutByte(uint8_t byte)
{
    /* 由于 IMU_buffer 与 IMU_bufferIdx 为 volatile，并由 ISR 修改，
       此函数可以在 ISR 中直接调用 */
   // App_USART6_Printf("RX: 0x%02X\n", byte);   // 打印每个接收到的字节
    if (IMU_bufferIdx == 0u) {
        /* 0x55 是帧头 */
        if (byte == 0x55u) {
            IMU_buffer[IMU_bufferIdx] = byte;
            IMU_bufferIdx = 1u;
        } else {
            /* 仍保持 0 */
            IMU_bufferIdx = 0u;
        }
    } else if (IMU_bufferIdx == 1u) {
        /* 0x53 是角度帧标识 */
        if (byte == 0x53u) {
            IMU_buffer[IMU_bufferIdx] = byte;
            IMU_bufferIdx = 2u;
        } else {
            /* 非角度帧：重同步（可按需扩展支持其它帧ID） */
            IMU_bufferIdx = 0u;
        }
    } else if (IMU_bufferIdx < 10u) {
        IMU_buffer[IMU_bufferIdx] = byte;
        IMU_bufferIdx++;
    } else if (IMU_bufferIdx == 10u) {
        /* 当前位置接收到校验字节 */
        if (byte == IMU_CheckSum((const uint8_t *)IMU_buffer)) {
            IMU_SaveBuffer((const uint8_t *)IMU_buffer);//把IMU_buffer前10个字节解析并保存到全局变量中
          //  App_USART6_Printf("yaw=%.2f\n", yawAngle);   // hyf添加调试信息。打印当前yaw角
        }
        /* 无论校验结果如何，都重置以等待下一个帧 */
        IMU_bufferIdx = 0u;
    } else {
        IMU_bufferIdx = 0u;
    }
}

/* 任务安全快照读取（短临界区，保证三轴角度一致） */
void IMU_GetAnglesSnapshot(float *out_roll, float *out_pinch, float *out_yaw)
{
    // if (out_roll == NULL || out_pinch == NULL || out_yaw == NULL) return;
    //
    // __disable_irq(); /* CMSIS：短暂禁止中断（只做最小复制） */
    // *out_roll = rollAngle;
    // *out_pinch = pinchAngle;
    // *out_yaw = yawAngle;
    // __enable_irq();
    __disable_irq();
    if (out_roll != NULL) *out_roll = rollAngle;
    if (out_pinch != NULL) *out_pinch = pinchAngle;
    if (out_yaw != NULL) *out_yaw = yawAngle;
    __enable_irq();//hyf！在 Angle_Turn 中调用 IMU_GetAnglesSnapshot(NULL, NULL, &current) 时，由于前两个指针为 NULL，而该函数要求三个指针都不能为 NULL，导致函数直接返回，current 没有被赋值，所以一直为 0。
    //解决方法：修改 IMU_GetAnglesSnapshot 函数，允许部分指针为 NULL。
}

void IMU_GetAnglesCalibrated(float *out_roll, float *out_pinch, float *out_yaw)
{
    __disable_irq();

    if(out_roll != NULL)
        *out_roll = rollAngle - roll_offset;

    if(out_pinch != NULL)
        *out_pinch = pinchAngle - pinch_offset;

    if(out_yaw != NULL)
    {
        float yaw = yawAngle - yaw_offset;

        /* 防止 yaw 跳变 */
        if(yaw > 180.0f) yaw -= 360.0f;
        if(yaw < -180.0f) yaw += 360.0f;

        *out_yaw = yaw;
    }

    __enable_irq();
}
