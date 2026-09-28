#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

extern uint32_t SystemCoreClock;
void vAssertCalled(const char *file, int line);

/* 调度和时钟 */
#define configNUMBER_OF_CORES                    1
#define configUSE_PREEMPTION                     1
#define configUSE_TIME_SLICING                   1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1
#define configCPU_CLOCK_HZ                       (SystemCoreClock)
#define configTICK_RATE_HZ                       1000U
#define configTICK_TYPE_WIDTH_IN_BITS            TICK_TYPE_WIDTH_32_BITS
#define configMAX_PRIORITIES                     8
#define configMINIMAL_STACK_SIZE                 256U
#define configMAX_TASK_NAME_LEN                  16
#define configIDLE_SHOULD_YIELD                  1
#define configUSE_TICKLESS_IDLE                  0

/* 动态分配：与 heap_4.c 配套 */
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configSUPPORT_STATIC_ALLOCATION          1
#define configTOTAL_HEAP_SIZE                    (32U * 1024U)
#define configAPPLICATION_ALLOCATED_HEAP         0
#define configKERNEL_PROVIDED_STATIC_MEMORY      1
/* 常用同步功能 */
#define configUSE_TASK_NOTIFICATIONS             1
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              0
#define configUSE_COUNTING_SEMAPHORES            1
#define configQUEUE_REGISTRY_SIZE                0
#define configUSE_CO_ROUTINES                    0
#define configUSE_TIMERS                         0

/* 首次移植保留错误检测 */
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configUSE_MALLOC_FAILED_HOOK             1
#define configCHECK_FOR_STACK_OVERFLOW           2
#define configCHECK_HANDLER_INSTALLATION         1
#define configUSE_TRACE_FACILITY                 0
#define configGENERATE_RUN_TIME_STATS            0
#define configUSE_NEWLIB_REENTRANT               0

/* STM32F427 实现了 4 个 NVIC 优先级位 */
#define configPRIO_BITS                          4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY  15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* 由 port.c 直接导出启动向量表使用的函数名 */
#define vPortSVCHandler                          SVC_Handler
#define xPortPendSVHandler                       PendSV_Handler
#define xPortSysTickHandler                      SysTick_Handler

#define configASSERT(x)                          \
    do                                          \
    {                                           \
        if (!(x))                               \
            vAssertCalled(__FILE__, __LINE__);   \
    } while (0)

/* 本文任务与排查过程使用的可选 API */
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskDelayUntil                  1
#define INCLUDE_vTaskDelete                      1
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_uxTaskGetStackHighWaterMark       1

#endif