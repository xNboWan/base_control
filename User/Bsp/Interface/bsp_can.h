#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"

bool canStart(CAN_HandleTypeDef *hcan);
bool canSend(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *pdata, uint8_t len);

#endif