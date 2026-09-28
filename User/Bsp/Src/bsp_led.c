#include "bsp_led.h"
#include "stm32f4xx_hal.h"


void ledOpen(ledType led)
{
    if (led == RED) HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
    else if (led ==GREEN) HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);
    else return;
}

void ledClose(ledType led)
{
    if (led == RED) HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
    else if (led ==GREEN) HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);
    else return;
}

void ledToggle(ledType led)
{
    if (led == RED) HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
    else if (led ==GREEN) HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
    else return;
}