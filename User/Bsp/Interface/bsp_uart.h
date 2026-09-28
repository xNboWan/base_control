#ifndef BSP_UART_H
#define BSP_UART_H

#include "stm32f4xx_hal.h"

void uartSend(UART_HandleTypeDef *uart, void* tx_buf);

void uartReceive(UART_HandleTypeDef *uart, void * rx_buf);

#endif