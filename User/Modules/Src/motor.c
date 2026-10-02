/**
 * @file motor.c
 * @author 李嘉羽 (aa01082241015@gmail.com)
 * @brief 电机抽象层实现
 * @version 0.1
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "motor.h"
#include "static_mem.h"


bool motorInit(motor_t *motor)
{
    return motor->ops->init(motor->ctx);
}

bool motorRead(motor_t *motor, motorData_t *pdata)
{
    return motor->ops->read(motor->ctx, pdata);
}

bool motorWrite(motor_t *motor, motorCmd_t *cmd)
{
    return motor->ops->write(motor->ctx, cmd);
}

bool motorSend(motor_t *motor)
{
    return motor->ops->send(motor->ctx);
}