#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"

/* 可以在调试器中查看，也可以在 vAssertCalled 上打断点。 */
const char *volatile g_rtos_assert_file;
volatile int g_rtos_assert_line;
volatile TaskHandle_t g_rtos_overflow_task;

void vAssertCalled(const char *file, int line)
{
    __disable_irq();
    g_rtos_assert_file = file;
    g_rtos_assert_line = line;

    for (;;)
        __NOP();
}

void vApplicationMallocFailedHook(void)
{
    vAssertCalled(__FILE__, __LINE__);
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task_name;
    g_rtos_overflow_task = task;
    vAssertCalled(__FILE__, __LINE__);
}