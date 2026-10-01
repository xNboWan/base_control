#include "motor.h"
#include "static_mem.h"


bool motorInit(motor_t *motor, uint8_t motor_num)
{
    bool pass = true;
    for (int i = 0; i < motor_num; i++)
    {
        pass &= motor[i].ops->init(motor[i].ctx);
    }
    return pass;
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