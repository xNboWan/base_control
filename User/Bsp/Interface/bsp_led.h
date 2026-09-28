#ifndef BSP_LED_H
#define BSP_LED_H

typedef enum{
    RED = 0,
    GREEN
} ledType;

void ledOpen(ledType led);
void ledToggle(ledType led);
void ledClose(ledType led);
#endif