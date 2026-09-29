#include "usart.h"
#include "../User_gx/Inc/JY901S.h"
#include "stm32f4xx_hal.h" /* for HAL handler types */

// /*
//  *  中断回调函数实现
//  *  - 对 huart1：在 HAL_UART_RxCpltCallback 中处理（接收到一个字节，传给 IMU_PutByte），
//  *    并再次调用 HAL_UART_Receive_DMA(&huart1, &IMU_uartRxBuffer, 1);
//
//  */
// //DMA目标地址
// volatile uint8_t IMU_uartRxBuffer;
//
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if(huart->Instance == huart1.Instance)
//     {
//         IMU_PutByte(IMU_uartRxBuffer);
//         HAL_UART_Receive_DMA(&huart1,&IMU_uartRxBuffer,1);
//     }
// }




