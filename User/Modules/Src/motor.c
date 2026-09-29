#include "motor.h"
#include "static_mem.h" 

bool motorInit(motor_t *motor)
{
    return motor->motor_ops->init;
}

bool motorRead(motor_t *motor, motorData_t *data)
{
    return motor->motor_ops->read;
}

bool motorWrite(motor_t *motor, motorCmd_t *cmd)
{
    return motor->motor_ops->write;
}