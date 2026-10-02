/**
 * @file bsp_can.h
 * @author 李嘉羽 (aa01082241015@gmail.com)
 * @brief CAN 底层驱动接口文件
 * @version 0.1
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"

bool canStart(CAN_HandleTypeDef *hcan);
bool canSend(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *pdata, uint8_t len);

#endif