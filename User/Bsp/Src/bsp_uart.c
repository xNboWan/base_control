#include "bsp_uart.h"

#define DELAY_T 10  /* ms */

void uartSend(UART_HandleTypeDef *huart, void* tx_buf)
{
    HAL_UART_Transmit(huart, tx_buf, 1, DELAY_T);
}

void uartReceive(UART_HandleTypeDef *huart, void * rx_buf)
{
    HAL_UART_Receive(huart, rx_buf, 1, DELAY_T);
}