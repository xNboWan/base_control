#include "chassis.h"
#include "static_mem.h"

#include "can.h"

#include "motor.h"
#include "m3508.h"

motor_t motor[4];

static m3508Ctx_t m3508_ctx[4];

static TaskHandle_t chassis_task_handle;

bool chassisInit(void)
{
    for (int i = 0; i < 4; i++)
    {
        m3508_ctx[i].hcan = &hcan1;
        m3508_ctx[i].id = i + 1;

        motor[i].ops = &m3508_ops;
        motor[i].ctx = &m3508_ctx[i];
    }
    if (!motorInit(motor, 4))
        return false;

    STATIC_MEM_TASK_ALLOC(chassisTask, 512);
    chassis_task_handle = STATIC_MEM_TASK_CREATE(chassisTask, chassisTask, "CHASSIS", NULL, 1);

    if (chassis_task_handle != NULL)
        return true;
    return false;
}

void chassisTask(void *arg)
{
    (void)arg;

    uint8_t select_motor = 0;

    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t last_switch_time = last_wake_time;

    const TickType_t switch_period = pdMS_TO_TICKS(3000);
    const TickType_t send_period = pdMS_TO_TICKS(2);

    static volatile uint32_t write_errors = 0;
    static volatile uint32_t send_errors = 0;

    for (;;)
    {
        TickType_t now = xTaskGetTickCount();

        // 每五秒切换到下一个电机
        if ((TickType_t)(now - last_switch_time) >= switch_period)
        {
            select_motor = (select_motor + 1) % 4;
            last_switch_time = now;
        }

        // 其他电机给零电流，选中的电机给 500
        motorCmd_t cmd[4] = {0};
        cmd[RR].torque = 0;

        // 更新四个电机的指令缓存
        for (uint8_t i = 0; i < 4; i++)
        {
            if (!motorWrite(&motor[i], &cmd[i]))
                write_errors++;
        }

        // 统一发送 ID 1～4 的指令
        if (!motorSend(&motor[FL]))
            send_errors++;

        vTaskDelayUntil(&last_wake_time, send_period);
    }
}