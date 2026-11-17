#ifndef __APP_RTOS_H__
#define __APP_RTOS_H__

#include <stdint.h>
#include "FreeRTOS.h"
#include "message_buffer.h"

#define RS485_MAX_LEN        256    // RS485 单包最大长度
#define RS485_MSGBUF_SIZE    4096   // RS485 消息缓冲总大小（约可容纳 15 个满帧）

void App_RTOS_Init(void);
void RS485_Send_Data(uint8_t *data, uint16_t len);

/* 由 HAL_UART_ErrorCallback（实现只能有一份，放在 esp8266.c）调用：
 * USART2 出 FE/ORE/NE 后 HAL 已经把接收 DMA 关掉了，这里负责重新挂上。
 * 不加这个，线上一出现一次帧错误，RS485 就永久失聪。 */
void RS485_Uart_Error_Rearm(uint32_t err_code);

#endif
