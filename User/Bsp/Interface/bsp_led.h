#ifndef BSP_LED_H
#define BSP_LED_H

#include "stm32f4xx_hal.h"

#define IMU_PASS   HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET)
#define ALL_PASS   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET)
#define INIT_ERROR HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET)
#endif