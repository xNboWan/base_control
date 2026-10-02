/**
 * @file wheel.c
 * @author 李嘉羽 (aa01082241015@gmail.com)
 * @brief 轮组模块实现
 * @version 0.1
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "wheel.h"
#include "generic_def.h"
#include "math.h"

bool wheelInit(wheel_t *wheel, wheelCfg_t *cfg)
{
    if (!wheel || !cfg || !isfinite(cfg->gear_ratio) || cfg->gear_ratio <= 0.0f
        || !isfinite(cfg->wheel_radius) || cfg->wheel_radius <= 0.0f || !isfinite(cfg->v_max)
        || cfg->v_max < 0.0f || (cfg->dir != FORWARD && cfg->dir != REVERSE)
        || cfg->pid_cfg.wrap != 0.0f)
        return false;

    CHECK(motorInit(&cfg->motor));
    wheel->cfg = *cfg;
    pidInit(&wheel->pid, &cfg->pid_cfg);

    return true;
};

bool wheelRead(wheel_t *wheel, wheelData_t *pdata)
{
    if (!wheel || !pdata)
        return false;
    motorData_t motor_data = {0};
    CHECK(motorRead(&wheel->cfg.motor, &motor_data));

    pdata->theta = motor_data.theta / wheel->cfg.gear_ratio;
    pdata->d_theta = wheel->cfg.dir * motor_data.d_theta / wheel->cfg.gear_ratio;
    pdata->target_d_theta = wheel->pid.setpoint;
    pdata->v = wheel->cfg.wheel_radius * pdata->d_theta;
    pdata->temperature = motor_data.temperature;
    wheel->data = *pdata;

    return true;
}

bool wheelWrite(wheel_t *wheel, wheelCmd_t *cmd, float dt)
{
    if (!wheel || !cmd || !isfinite(cmd->d_theta) || !isfinite(dt) || dt <= 0.0f
        || wheel->cfg.pid_cfg.wrap != 0)
        return false;

    motorCmd_t motor_cmd = {0};
    motorData_t motor_data = {0};

    if (!motorRead(&wheel->cfg.motor, &motor_data) || !isfinite(motor_data.d_theta))
    {
        pidReset(&wheel->pid);
        motor_cmd.lq = 0.0f;
        CHECK(motorWrite(&wheel->cfg.motor, &motor_cmd));
        return false;
    }

    float speed = wheel->cfg.dir * motor_data.d_theta / wheel->cfg.gear_ratio;

    float target = cmd->d_theta;
    float limit = wheel->cfg.v_max / wheel->cfg.wheel_radius;
    if (target > limit)
        target = limit;
    else if (target < -limit)
        target = -limit;

    float current = pidCalc(&wheel->pid, target, speed, dt);

    motor_cmd.lq = wheel->cfg.dir * current;
    if (!motorWrite(&wheel->cfg.motor, &motor_cmd))
    {
        pidReset(&wheel->pid);
        motor_cmd.lq = 0.0f;
        CHECK(motorWrite(&wheel->cfg.motor, &motor_cmd));
        return false;
    }
    return true;
}

bool wheelSend(wheel_t *wheel)
{
    if (!wheel)
        return false;

    CHECK(motorSend(&wheel->cfg.motor));
    return true;
}

bool wheelGroupWriteAndSend(wheelGroup_t *wheel_group, wheelCmd_t *cmd, float dt)
{
    if (!wheel_group || !wheel_group->wheel || wheel_group->wheel_num == 0 || !cmd || !isfinite(dt)
        || dt <= 0.0f)
        return false;

    uint8_t wheel_num = wheel_group->wheel_num;

    bool pass = true;
    for (uint8_t i = 0; i < wheel_num; i++)
    {
        if (!wheelWrite(&wheel_group->wheel[i], &cmd[i], dt))
        {
            pass = false;
            motorCmd_t motor_cmd = {0};
            pidReset(&wheel_group->wheel[i].pid);
            CHECK(motorWrite(&wheel_group->wheel[i].cfg.motor, &motor_cmd));
        }
    }

    CHECK(wheelSend(wheel_group->wheel));

    return pass;
}
