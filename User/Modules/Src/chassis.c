/**
 * @file chassis.c
 * @author 李嘉羽 (aa01082241015@gmail.com)
 * @brief 底盘模块实现
 * @version 0.1
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "chassis.h"
#include "static_mem.h"

#include "can.h"

#include "motor.h"
#include "wheel.h"
#include "m3508.h"
#include "pid.h"
#include "generic_def.h"


static m3508Ctx_t m3508_ctx[4];
motor_t motor[4];
wheel_t wheel[4];
static wheelGroup_t wheel_group;
static TaskHandle_t chassis_task_handle;

bool chassisInit(void)
{
    m3508_ctx[FL].hcan = &hcan1;
    m3508_ctx[FL].id = 1;
    m3508_ctx[FR].hcan = &hcan1;
    m3508_ctx[FR].id = 2;
    m3508_ctx[RL].hcan = &hcan1;
    m3508_ctx[RL].id = 3;
    m3508_ctx[RR].hcan = &hcan1;
    m3508_ctx[RR].id = 4;
    
    motor[FL].ctx = &m3508_ctx[FL];
    motor[FR].ctx = &m3508_ctx[FR];
    motor[RL].ctx = &m3508_ctx[RL];
    motor[RR].ctx = &m3508_ctx[RR];
    motor[FL].ops = &m3508_ops;
    motor[FR].ops = &m3508_ops;
    motor[RL].ops = &m3508_ops;
    motor[RR].ops = &m3508_ops;
    
    pidCfg_t pid_cfg = 
    {
        .kp = 5.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .kff = 0.0f,
        .out_min = -10000.0f,
        .out_max = 10000.0f,
        .integral_max = 10000.0f,
        .d_filter_alpha = 0.0f,
        .wrap = 0.0f
    };

    wheelCfg_t wheel_cfg_template;
    
    wheel_cfg_template.motor = motor[FL];
    wheel_cfg_template.wheel_radius = WHEEL_RADIUS;
    wheel_cfg_template.gear_ratio = GEAR_RATIO;
    wheel_cfg_template.dir = FORWARD;
    wheel_cfg_template.pid_cfg = pid_cfg;
    wheel_cfg_template.v_max = 100.0f;
    
    wheelCfg_t wheel_cfg[4];
    wheel_cfg[FL] = wheel_cfg_template;
    wheel_cfg[FR] = wheel_cfg_template;
    wheel_cfg[RL] = wheel_cfg_template;
    wheel_cfg[RR] = wheel_cfg_template;
    wheel_cfg[FL].motor = motor[FL];
    wheel_cfg[FR].motor = motor[FR];
    wheel_cfg[RL].motor = motor[RL];
    wheel_cfg[RR].motor = motor[RR];
    wheel_cfg[FL].dir = REVERSE;
    wheel_cfg[FR].dir = FORWARD;
    wheel_cfg[RL].dir = REVERSE;
    wheel_cfg[RR].dir = FORWARD;

    wheel_group.wheel = wheel;
    wheel_group.wheel_num = 4;

    wheelInit(&wheel[FL], &wheel_cfg[FL]);
    wheelInit(&wheel[FR], &wheel_cfg[FR]);
    wheelInit(&wheel[RL], &wheel_cfg[RL]);
    wheelInit(&wheel[RR], &wheel_cfg[RR]);

    STATIC_MEM_TASK_ALLOC(chassisTask, 512);
    chassis_task_handle = STATIC_MEM_TASK_CREATE(chassisTask, chassisTask, "CHASSIS", NULL, 1);

    if (chassis_task_handle != NULL)
        return true;
    return false;
}

void chassisTask(void *arg)
{
    (void)arg;
    TickType_t last_wake_time = xTaskGetTickCount();
    const float dt = 0.001f;

    for (;;)
    {
        wheelCmd_t cmd[4];
        for (uint8_t i = 0; i < 4; i++) cmd[i].d_theta = PI;

        wheelGroupWriteAndSend(&wheel_group, cmd, dt);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000));
    }
}