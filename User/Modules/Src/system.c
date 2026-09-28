#include "system.h"
#include "imu.h"
#include "saber.h"
#include "usart.h"
#include "bsp_led.h"
#include "static_mem.h"

saberCtx_t saber_ctx =
{
    .huart = &huart6,
};

imu_t imu =
{
    .ops = &saber_ops,
    .ctx = &saber_ctx,
};

static void systemTask(void* arg);

STATIC_MEM_TASK_ALLOC(systemTask, 256);

void systemLaunch(void)
{
    STATIC_MEM_TASK_CREATE(systemTask, systemTask, "SYSTEM", NULL, 3);
}

void systemTask(void* arg)
{
    (void)arg;
    if (imuInit(&imu)) ledOpen(GREEN );

    while(1)
    vTaskDelay(portMAX_DELAY);
}