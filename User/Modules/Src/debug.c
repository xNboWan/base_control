#include "debug.h"
#include "static_mem.h"
#include "imu.h"
#include "common.h"

static TaskHandle_t debug_task_handle;

static void debugTask(void *arg);

static float acc[3];
static float gyro[3];
static float yaw;

bool debugInit(void)
{
    STATIC_MEM_TASK_ALLOC(debugTask, 256);
    debug_task_handle = STATIC_MEM_TASK_CREATE(debugTask, debugTask, "DEBUG", NULL, 1);
    if (debug_task_handle != NULL)
        return true;
    else
        return false;
}

void debugTask(void *arg)
{
    (void)arg;
    imuData_t data;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    for (;;)
    {
        imuRead(&imu, &data);
        for (uint8_t i = 0; i < 3; i++)
        {
            acc[i] = data.accel[i];
            gyro[i] = data.gyro[i];
        }

        yaw = data.yaw;

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}