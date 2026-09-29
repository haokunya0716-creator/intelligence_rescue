/*
* app_usart2.h
 *
 *  Created on: May 5, 2025
 *      Author: gaoxi
 */

#ifndef INC_APP_USART_H_
#define INC_APP_USART_H_

#include <stdint.h>

typedef enum
{
    APP_USART_USB_OK = 0,
    APP_USART_USB_NOT_READY,
    APP_USART_USB_QUEUE_FULL,
    APP_USART_USB_TOO_LONG,
    APP_USART_USB_FORMAT_ERROR
} AppUsartUsbStatus;

typedef struct
{
    uint32_t sent_count;
    uint32_t queue_drop_count;
    uint32_t not_ready_count;
    uint32_t too_long_count;
    uint32_t format_error_count;
    uint32_t send_error_count;
} AppUsartUsbStats;

void App_USART2_Printf(const char* Format, ...);
void App_USART1_Printf(const char* Format, ...);
void App_USART6_Printf(const char* Format, ...);

AppUsartUsbStatus App_Usart_USB(const char *format, ...);
void App_Usart_USB_Process(void);
void App_Usart_USB_GetStats(AppUsartUsbStats *stats);

/*
 * Called by the USB CDC transmit-complete callback. Application code should
 * not call this function directly.
 */
void App_Usart_USB_OnTxComplete(void);

#endif /* INC_APP_USART_H_ */
