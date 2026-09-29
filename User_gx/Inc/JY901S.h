#ifndef __JY901S_H
#define __JY901S_H


#include <stdint.h>

/*
 *  C 版本 IMU 接口
 *
 * 变量说明：
 *  - IMU_rawRoll / IMU_rawPinch / IMU_rawYaw : 原始有符号 16-bit 值（int16_t）
 *  - rollAngle / pinchAngle / yawAngle : 对应角度，单位：度（float）
 *  - IMU_buffer / IMU_bufferIdx : 内部接收缓冲（volatile，因为在中断回调函数内会修改）
 *
 * 函数：
 *  - void IMU_PutByte(uint8_t b) : 逐字节输入，由串口回调/ISR 调用
 *  - uint8_t IMU_CheckSum(const uint8_t *buf) : 计算校验（前 10 字节求和并取低 8 位）
 *  - void IMU_GetAnglesSnapshot(float *out_roll, float *out_pinch, float *out_yaw) :
 *      在短临界区内复制当前角度，供任务安全读取
 */

extern volatile int16_t IMU_rawRoll;
extern volatile int16_t IMU_rawPinch;
extern volatile int16_t IMU_rawYaw;

extern volatile float rollAngle;
extern volatile float pinchAngle;
extern volatile float yawAngle;

extern volatile uint8_t IMU_buffer[12];
extern volatile uint8_t IMU_bufferIdx;

/* 把串口接收到的单字节传入 IMU 解析器（ISR 中调用） */
void IMU_PutByte(uint8_t byte);

/* 计算校验：对 buf[0..9] 求和并取低 8 位（与原实现一致） */
uint8_t IMU_CheckSum(const uint8_t *buf);

/* 任务安全读取当前角度的快照（在短临界区复制） */
void IMU_GetAnglesSnapshot(float *out_roll, float *out_pinch, float *out_yaw);
/* 角度零点校准 */
void IMU_CalibrateOffset(void);

/* 获取校准后的角度 */
void IMU_GetAnglesCalibrated(float *out_roll, float *out_pinch, float *out_yaw);
#endif /* __JY901S_H */
