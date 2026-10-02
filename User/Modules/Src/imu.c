/**
 * @file imu.c
 * @author 李嘉羽 (aa01082241015@gmail.com)
 * @brief IMU 采样模块实现
 * @version 0.1
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "imu.h"
#include "static_mem.h"

static QueueHandle_t sampleQueue;
static TaskHandle_t imu_task_handle;
static bool is_init = false;

static void imuTask(void *arg);

bool imuInit(imu_t *imu)
{
    if (!is_init)
    {
        STATIC_MEM_QUEUE_ALLOC(sampleQueue, 1, sizeof(imuData_t));
        sampleQueue = STATIC_MEM_QUEUE_CREATE(sampleQueue);

        STATIC_MEM_TASK_ALLOC(imuTask, 256);
        imu_task_handle = STATIC_MEM_TASK_CREATE(imuTask, imuTask, "IMU", imu, 1);
        is_init = true;
    }
    return imu->ops->init(imu->ctx);
}

bool imuRead(imu_t *imu, imuData_t *pdata)
{
    return xQueuePeek(sampleQueue, pdata, 0) == pdTRUE;
}

void imuTask(void *arg)
{
    imu_t *imu = arg;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        imuData_t sample = {0};
        if(imu->ops->read(imu->ctx, &sample)) xQueueOverwrite(sampleQueue, &sample);    
        else vTaskDelay(1);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}
