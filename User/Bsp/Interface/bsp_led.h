/**
 * @file bsp_led.h
 * @author 李嘉羽 (aa01082241015@gmail.com)
 * @brief 使用led宏来判断模块初始化是否通过
 * @version 0.1
 * @date 2026-09-26
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef BSP_LED_H
#define BSP_LED_H

#include "stm32f4xx_hal.h"

#define IMU_PASS   HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET)
#define DEBUG_PASS HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET)
#define CHASSIS_PASS HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET)
#define REMOTE_PASS HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET)

#define ALL_PASS   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET)
#define INIT_ERROR HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET)

#endif