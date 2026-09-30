#include "stdio.h"

#include "bsp_uart.h"
#include "stm32f4xx_hal.h"
#include "usart.h"

int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart8, (uint8_t*)ptr, (uint16_t)len, 10);
    return len;
}

