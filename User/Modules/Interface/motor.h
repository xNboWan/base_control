#ifndef MOTOR_H
#define MOTOR_H

#include "stdbool.h"
#include "stdint.h"

typedef enum
{
    FL = 0,
    FR,
    RL,
    RR
} motorSequence_e;

typedef struct
{
    float omega;            // 机械角度 pi
    float d_omega;          // 角速度  pi/s
    float lq;               // 转矩电流 A
    int16_t temperature;    // 温度 
} motorData_t;

typedef struct
{
    int16_t torque;  // 力矩 N*m/s
} motorCmd_t;

typedef struct
{
    bool (*init)(void *ctx);
    bool (*read)(void *ctx, motorData_t *pdata);
    bool (*write)(void *ctx, motorCmd_t *cmd);
    bool (*send)(void *ctx);
} motorOps_t;

typedef struct
{
    motorOps_t *ops;
    void *ctx;
} motor_t;

bool motorInit(motor_t *motor, uint8_t motor_num);
bool motorRead(motor_t *motor, motorData_t *pdata);
bool motorWrite(motor_t *motor, motorCmd_t *cmd);
bool motorSend(motor_t *motor);

extern motor_t motor[4];

#endif