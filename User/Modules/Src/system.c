/**
 * @file system.c
 * @author 李嘉羽 (aa01082241015@gmail.com)
 * @brief 系统启动模块实现
 * @version 0.1
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "usart.h"
#include "can.h"

#include "bsp_led.h"
#include "imu.h"
#include "saber.h"
#include "chassis.h"

#include "static_mem.h"
#include "system.h"
#include "debug.h"

#define PASS(NAME, ARG, MODULE) do { \
                bool step_ok = NAME##Init(ARG); \
                pass &= step_ok; \
                if (step_ok) { MODULE##_PASS; } \
                vTaskDelay(pdMS_TO_TICKS(500));} while (0)

                
saberCtx_t saber_ctx = {
    .huart = &huart6,
};

imu_t imu = {
    .ops = &saber_ops,
    .ctx = &saber_ctx,
};


static void systemTask(void *arg);

void systemLaunch(void)
{
    STATIC_MEM_TASK_ALLOC(systemTask, 256);
    STATIC_MEM_TASK_CREATE(systemTask, systemTask, "SYSTEM", NULL, 3);
}

void systemTask(void *arg)
{
    (void)arg;
    bool pass = true;

    pass &= imuInit(&imu);  if (pass) IMU_PASS;
    pass &= debugInit();    if (pass) DEBUG_PASS;
    pass &= chassisInit();  if (pass) CHASSIS_PASS;

    if (pass)
    { 
        ALL_PASS; 
        while (1) vTaskDelay(portMAX_DELAY);
    }
    else
    { 
        INIT_ERROR;
    }
}