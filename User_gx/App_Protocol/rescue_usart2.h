#ifndef RESCUE_USART2_H
#define RESCUE_USART2_H

#include <stdint.h>
#include "rescue_protocol.h"

/* MX_USART2_UART_Init() 后调用一次，开始 USART2 单字节中断接收。 */
uint8_t Rescue_Usart2_Start(void);

/* 只在 HAL_UART_RxCpltCallback() 的 USART2 分支中调用。 */
void Rescue_Usart2_RxCallback(void);

/*
 * 主循环每次迭代都调用：处理收到的命令、发送 ACK、运行当前选中的闭环。
 * 此函数在主循环运行，可能通过阻塞式 HAL UART 发送短 ACK 帧。
 */
void Rescue_Usart2_Process(void);

/* 通用帧发送接口，主要用于状态上报或联调；成功返回 1。 */
uint8_t Rescue_Usart2_SendFrame(const RescueFrame *frame);

uint32_t Rescue_Usart2_DroppedCount(void);
uint32_t Rescue_Usart2_ParseErrorCount(void);
uint32_t Rescue_Usart2_AckErrorCount(void);

#endif
