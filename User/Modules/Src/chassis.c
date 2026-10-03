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

#include "math.h"

#include "chassis.h"
#include "can.h"
#include "static_mem.h"

#include "motor.h"
#include "imu.h"
#include "wheel.h"
#include "m3508.h"
#include "pid.h"
#include "generic_def.h"

typedef struct
{
    float vx;
    float vy;
} vec2;

typedef struct
{
    float x;
    float y;
    float yaw;
} frame;

static m3508Ctx_t m3508_ctx[4];
motor_t motor[4];
wheel_t wheel[4];

static wheelGroup_t wheel_group;
static chassisCmd_t chassis_cmd = {.mode = HEADFREE};
static pidController_t yaw_pid;
static TaskHandle_t chassis_task_handle;
static frame world_frame = {0};
static frame machine_frame;

static bool is_init = false;

static void chassisInverseKinematicsSolution(const chassisCmd_t *target, wheelCmd_t wheel_cmd[4]);
void coordinateFrameTransformation(frame *from, frame *to, vec2 *input, vec2 *output);

bool chassisInit(void)
{
    if (is_init) return true;

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

    static const pidCfg_t pid_cfg = {
        .kp = 5.5f,
        .ki = 0.01f,
        .kd = 0.01f,
        .kff = 0.0f,
        .out_min = -20.0f,
        .out_max = 20.0f,
        .integral_max = 1000.0f,
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

    static const pidCfg_t yaw_pid_cfg =
    {
    .kp = 0.0f,
    .ki = 0.01f,
    .kd = 0.0f,
    .kff = 0.0f,
    .out_min = -1.0f,       /* rad/s */
    .out_max =  1.0f,
    .integral_max = 10.0f,   /* I 项输出上限，rad/s */
    .d_filter_alpha = 0.2f,
    .wrap = PI,
    };

    pidInit(&yaw_pid, &yaw_pid_cfg);

    STATIC_MEM_TASK_ALLOC(chassisTask, 1024);
    chassis_task_handle = STATIC_MEM_TASK_CREATE(chassisTask, chassisTask, "CHASSIS", NULL, 1);

    if (!chassis_task_handle)
        return false;
    is_init = true;
    return true;
}

bool chassisSetCommand(const chassisCmd_t *cmd)
{
    if (!is_init || !cmd || (cmd->mode != HEADLOCK && cmd->mode != HEADFREE) ||
        !isfinite(cmd->vx) || !isfinite(cmd->vy) ||
        !isfinite(cmd->yaw) || !isfinite(cmd->d_yaw))
        return false;

    taskENTER_CRITICAL();
    chassis_cmd = *cmd;
    taskEXIT_CRITICAL();
    return true;
}

void chassisTask(void *arg)
{
    (void)arg;

    TickType_t last_wake_time = xTaskGetTickCount();
    const float dt = 0.001f;
    wheelCmd_t wheel_cmd[4] = {0};

    const vec2 world_velocity = {
        .vx = 1.0f,
        .vy = 0.0f
    };

    for (;;)
    {
        chassisCmd_t target = {
            .mode = HEADFREE
        };
        imuData_t imu_data = {0};

        if (imuRead(&imu, &imu_data) && isfinite(imu_data.yaw))
        {
            machine_frame.yaw = imu_data.yaw;

            vec2 machine_velocity = {0};
            vec2 input = world_velocity;

            coordinateFrameTransformation(&world_frame,
                                          &machine_frame,
                                          &input,
                                          &machine_velocity);

            target.vx = machine_velocity.vx;
            target.vy = machine_velocity.vy;
            target.d_yaw = 1.0f;  
        }

        chassisInverseKinematicsSolution(&target, wheel_cmd);
        wheelGroupWriteAndSend(&wheel_group, wheel_cmd, dt);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
}

void chassisInverseKinematicsSolution(const chassisCmd_t *target, wheelCmd_t wheel_cmd[4])
{
    if (!wheel_cmd)
        return;

    for (uint8_t i = 0; i < 4; i++)
        wheel_cmd[i].d_theta = 0.0f;

    if (!target || !isfinite(target->vx) || !isfinite(target->vy) || !isfinite(target->d_yaw))
        return;

    const float vx = target->vx;
    const float vy = target->vy;
    const float rotation = (Rx + Ry) * target->d_yaw;
    const float inv_r = 1.0f / WHEEL_RADIUS;
    const float d_theta[4] = {
        [FL] = (vx - vy - rotation) * inv_r,
        [FR] = (vx + vy + rotation) * inv_r,
        [RL] = (vx + vy - rotation) * inv_r,
        [RR] = (vx - vy + rotation) * inv_r,
    };

    for (uint8_t i = 0; i < 4; i++)
    {
        if (!isfinite(d_theta[i]))
            return;
    }

    for (uint8_t i = 0; i < 4; i++)
        wheel_cmd[i].d_theta = d_theta[i];
}

void coordinateFrameTransformation(frame *from, frame *to,
                                   vec2 *input, vec2 *output)
{
    const float angle = from->yaw - to->yaw;
    const float c = cosf(angle);
    const float s = sinf(angle);

    /* 缓存输入，允许 input 和 output 指向同一个对象 */
    const float vx = input->vx;
    const float vy = input->vy;

    output->vx = c * vx - s * vy;
    output->vy = s * vx + c * vy;
}