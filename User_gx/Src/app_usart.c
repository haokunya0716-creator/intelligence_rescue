/*
 * app_usart.c
 *
 * Created on: May 5, 2025
 * Author: gaoxi
 */

#include <stdarg.h>
#include <stdint.h>
#include "main.h"
#include "usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "app_usart.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern USBD_HandleTypeDef hUsbDeviceFS;

#define APP_USART_FORMAT_BUFFER_SIZE 128U
#define APP_USART_USB_QUEUE_DEPTH    8U
#define APP_USART_USB_MESSAGE_SIZE   128U

typedef struct
{
    uint8_t data[APP_USART_USB_MESSAGE_SIZE];
    uint16_t length;
} AppUsartUsbMessage;

static AppUsartUsbMessage usb_queue[APP_USART_USB_QUEUE_DEPTH];
static volatile uint8_t usb_queue_head = 0U;
static volatile uint8_t usb_queue_tail = 0U;
static volatile uint8_t usb_queue_count = 0U;
static volatile uint8_t usb_tx_active = 0U;
static volatile AppUsartUsbStats usb_stats = {0};

static uint8_t App_Usart_USB_IsReady(void)
{
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) &&
           (hUsbDeviceFS.pClassData != NULL);
}

void App_USART1_Printf(const char* Format, ...)
{
    char format_buffer[APP_USART_FORMAT_BUFFER_SIZE];

    va_list argptr;

    va_start(argptr, Format);

    (void)vsnprintf(format_buffer, sizeof(format_buffer), Format, argptr);

    va_end(argptr);

    HAL_UART_Transmit(&huart1, (uint8_t *)format_buffer, strlen(format_buffer), HAL_MAX_DELAY);
}

//
// @简介：使用串口2打印格式化字符串
//用法与printf差不多
//
void App_USART2_Printf(const char* Format, ...)
{
    char format_buffer[APP_USART_FORMAT_BUFFER_SIZE];

    va_list argptr;

    va_start(argptr, Format);

    (void)vsnprintf(format_buffer, sizeof(format_buffer), Format, argptr);

    va_end(argptr);

    HAL_UART_Transmit(&huart2, (uint8_t *)format_buffer, strlen(format_buffer), HAL_MAX_DELAY);
}

void App_USART6_Printf(const char* Format, ...)
{
    char format_buffer[APP_USART_FORMAT_BUFFER_SIZE];

    va_list argptr;

    va_start(argptr, Format);

    (void)vsnprintf(format_buffer, sizeof(format_buffer), Format, argptr);

    va_end(argptr);

    HAL_UART_Transmit(&huart6, (uint8_t *)format_buffer, strlen(format_buffer), HAL_MAX_DELAY);
}

AppUsartUsbStatus App_Usart_USB(const char *format, ...)
{
    char format_buffer[APP_USART_USB_MESSAGE_SIZE];
    int formatted_length;
    va_list argptr;

    if (format == NULL)
    {
        usb_stats.format_error_count++;
        return APP_USART_USB_FORMAT_ERROR;
    }

    if (!App_Usart_USB_IsReady())
    {
        usb_stats.not_ready_count++;
        return APP_USART_USB_NOT_READY;
    }

    va_start(argptr, format);
    formatted_length = vsnprintf(format_buffer,
                                  sizeof(format_buffer),
                                  format,
                                  argptr);
    va_end(argptr);

    if (formatted_length < 0)
    {
        usb_stats.format_error_count++;
        return APP_USART_USB_FORMAT_ERROR;
    }

    if ((size_t)formatted_length >= sizeof(format_buffer))
    {
        usb_stats.too_long_count++;
        return APP_USART_USB_TOO_LONG;
    }

    /*
     * Formatting is done outside the critical section. Only the short queue
     * update is protected because the USB completion callback runs in an ISR.
     */
    __disable_irq();
    if (usb_queue_count >= APP_USART_USB_QUEUE_DEPTH)
    {
        __enable_irq();
        usb_stats.queue_drop_count++;
        return APP_USART_USB_QUEUE_FULL;
    }

    (void)memcpy(usb_queue[usb_queue_head].data,
                 format_buffer,
                 (size_t)formatted_length);
    usb_queue[usb_queue_head].length = (uint16_t)formatted_length;
    usb_queue_head++;
    if (usb_queue_head >= APP_USART_USB_QUEUE_DEPTH)
    {
        usb_queue_head = 0U;
    }
    usb_queue_count++;
    __enable_irq();

    return APP_USART_USB_OK;
}

void App_Usart_USB_Process(void)
{
    uint8_t queue_index;
    uint8_t transmit_result;

    __disable_irq();
    if (!App_Usart_USB_IsReady())
    {
        /*
         * A disconnect may prevent the CDC completion callback from running.
         * Release the active flag but keep the current queue item so it can
         * be retried after the host reconnects.
         */
        usb_tx_active = 0U;
        __enable_irq();
        return;
    }

    if ((usb_tx_active != 0U) || (usb_queue_count == 0U))
    {
        __enable_irq();
        return;
    }

    queue_index = usb_queue_tail;
    usb_tx_active = 1U;
    __enable_irq();

    transmit_result = CDC_Transmit_FS(usb_queue[queue_index].data,
                                      usb_queue[queue_index].length);

    if (transmit_result == USBD_OK)
    {
        return;
    }

    /*
     * BUSY means the USB stack needs another opportunity. A hard failure
     * cannot be retried safely forever, so discard this message.
     */
    __disable_irq();
    usb_tx_active = 0U;
    if (transmit_result != USBD_BUSY)
    {
        if (usb_queue_count != 0U)
        {
            usb_queue_tail++;
            if (usb_queue_tail >= APP_USART_USB_QUEUE_DEPTH)
            {
                usb_queue_tail = 0U;
            }
            usb_queue_count--;
        }
        usb_stats.send_error_count++;
        usb_stats.queue_drop_count++;
    }
    __enable_irq();
}

void App_Usart_USB_OnTxComplete(void)
{
    if (usb_tx_active == 0U)
    {
        return;
    }

    usb_tx_active = 0U;
    if (usb_queue_count != 0U)
    {
        usb_queue_tail++;
        if (usb_queue_tail >= APP_USART_USB_QUEUE_DEPTH)
        {
            usb_queue_tail = 0U;
        }
        usb_queue_count--;
    }
    usb_stats.sent_count++;
}

void App_Usart_USB_GetStats(AppUsartUsbStats *stats)
{
    if (stats == NULL)
    {
        return;
    }

    __disable_irq();
    *stats = usb_stats;
    __enable_irq();
}
