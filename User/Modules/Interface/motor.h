/**
 * @file motor.h
 * @author 李嘉羽 (aa01082241015@gmail.com)
 * @brief 电机抽象层接口文件
 * @version 0.1
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 */

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
    float theta;            // 机械角度 rad
    float d_theta;          // 角速度  rad/s
    float lq;               // 转矩电流 A
    int16_t temperature;    // 温度 
} motorData_t;

typedef struct
{
        float lq;           // 转矩电流 A
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

bool motorInit(motor_t *motor);
bool motorRead(motor_t *motor, motorData_t *pdata);
bool motorWrite(motor_t *motor, motorCmd_t *cmd);
bool motorSend(motor_t *motor);

extern motor_t motor[4];

#endif