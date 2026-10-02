/**
 * @file m3508.h
 * @author 李嘉羽 (aa01082241015@gmail.com)
 * @brief M3508 电机驱动接口文件
 * @version 0.1
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef M3508_H
#define M3508_H

#include "stm32f4xx_hal.h"
#include "motor.h"

typedef struct
{
    CAN_HandleTypeDef *hcan;
    uint8_t id;
} m3508Ctx_t;

extern motorOps_t m3508_ops;

#endif