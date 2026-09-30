#ifndef M3508_H
#define M3508_H

#include "stm32f4xx_hal.h"

typedef struct
{
    CAN_HandleTypeDef hcan;
    uint8_t id;
} m3508Ctx_t;


#endif