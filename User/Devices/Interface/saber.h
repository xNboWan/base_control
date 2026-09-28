/**
 * @file saber.h
 * @author 李嘉羽 (aa01082241015@gmail.com)
 * @brief saber惯导驱动接口文件
 * @version 0.1
 * @date 2026-09-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef SABER_H
#define SABER_H

#include "imu.h"
#include "stdbool.h"
#include "stm32f4xx_hal.h"

typedef struct
{
    UART_HandleTypeDef *huart;
} saberCtx_t;

extern saberCtx_t saber_ctx;

bool saberInit(void *ctx);
bool saberRead(void *ctx, imuData_t *data);

extern imuOps_t saber_ops;
#endif
