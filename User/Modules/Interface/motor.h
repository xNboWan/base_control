#ifndef MOTOR_H
#define MOTOR_H

#include "stdbool.h"
#include "stdint.h"

typedef struct
{
    int16_t omega;          // 机械角度 pi
    int16_t d_omega;      // 角速度  pi/s
    int16_t torque;         // 力矩 N*m/s
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
    bool (*write)(void *ctx, motorCmd_t *pdata);
} motorOps_t;

typedef struct
{
    motorOps_t *motor_ops;
    void *ctx;
} motor_t;

bool motorInit(motor_t *motor);
bool motorRead(motor_t *motor, motorData_t *pdata);
bool motorWrite(motor_t *motor, motorCmd_t *cmd);

extern motor_t motorFL;
extern motor_t motorFR;
extern motor_t motorRL;
extern motor_t motorRR;

#endif