/**
 * @file bsp_uart.c
 * @author 李嘉羽 (aa01082241015@gmail.com)
 * @brief UART 底层驱动实现（printf 重定向）
 * @version 0.1
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "stdio.h"

#include "bsp_uart.h"
#include "stm32f4xx_hal.h"
#include "usart.h"

int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart7, (uint8_t*)ptr, (uint16_t)len, 10);
    return len;
}

