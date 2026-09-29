#include "usart.h"

#include "bsp_led.h"
#include "imu.h"
#include "saber.h"
#include "static_mem.h"
#include "system.h"
#include "debug.h"

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

    pass &= imuInit(&imu);
    if (pass)
        IMU_PASS;

    pass &= debugInit();
    if (pass)
        DEBUG_PASS;

    if (pass)
        ALL_PASS;
    else
        INIT_ERROR;

    while (1)
        vTaskDelay(portMAX_DELAY);
}