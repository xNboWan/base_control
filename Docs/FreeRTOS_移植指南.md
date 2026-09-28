# STM32F427 工程 FreeRTOS 移植指南

本文针对 `base_control` 工程已经复制内核、尚未接入构建和启动流程的状态。先完成两个 LED 任务的调度验证，再接入 IMU、DMA 和控制任务。本文中的代码是待实施的示例，编写文档时没有修改工程源码或构建配置。

## 1. 确认本工程的移植组合

| 项目 | 当前工程情况 | 本文采用的方案 |
|---|---|---|
| MCU | `STM32F427IIH6`，Cortex-M4F | 单核、无 MPU 隔离 |
| 工具链 | `arm-none-eabi-gcc`，CMake + Ninja | GCC 端口 |
| 内核目录 | `MiddleWares/FreeRTOS/` | 注意目录实际拼写和大小写 |
| 内核版本 | `include/task.h` 中为 `V11.3.1` | 使用该版本的配置宏 |
| 系统时钟 | `.ioc` 中 HCLK 为 180 MHz | `configCPU_CLOCK_HZ` 使用 `SystemCoreClock` |
| 处理器端口 | 内核提供多个编译器和处理器端口 | 只编译 `portable/GCC/ARM_CM4F/port.c` |
| 动态内存 | 尚未选择 | 只编译 `portable/MemMang/heap_4.c` |
| HAL 时基 | `Core/Src/stm32f4xx_hal_timebase_tim.c` 使用 TIM14 | 保留 TIM14 驱动 HAL tick |
| FreeRTOS 时基 | 尚未接入 | SysTick，示例为 1 kHz |
| 异常入口 | `stm32f4xx_it.c` 中有空的 SVC、PendSV、SysTick | 由 FreeRTOS 端口直接提供这三个入口 |
| DMA RX | USART6 RX 使用 DMA2 Stream1，当前 IRQ 优先级为 0 | 调用 `FromISR` API 前改为允许的优先级 |

本方案直接使用 FreeRTOS 原生 API，不需要额外引入 `cmsis_os.h`、`cmsis_os2.h` 或 CMSIS-RTOS 适配层。只复制内核不会自动生成 `FreeRTOSConfig.h`、创建任务或启动调度器。

## 2. 先建立一个可排查的验证基线

1. 确认当前 GPIO、系统时钟和 LED 驱动可以正常工作。
2. 首次验证时，先从 `main()` 的启动路径中移开 IMU 初始化及业务循环，只启动本文的两个 LED 任务。避免 IMU 握手等待使程序根本没有走到调度器。
3. 当前 CMake 会自动收集 `User/` 下所有 `.c`。如果某个正在开发的驱动仍有未完成表达式、未定义变量或缺失定义，需要先使它能编译，或者在验证期间明确从构建中排除。仅注释掉函数调用，不能消除源文件本身的语法错误。
4. 不要把已有裸机编译错误归因于 FreeRTOS；先区分“源文件编译失败”“内核链接失败”和“调度器运行异常”。

建议最终需要新增或调整的文件如下。表格只是操作清单，不表示已经完成这些改动。

| 文件 | 操作 |
|---|---|
| `CMakeLists.txt` | 加入选定的内核源文件和头文件路径 |
| `Core/Inc/FreeRTOSConfig.h` | 新建工程专用配置 |
| `Core/Src/stm32f4xx_it.c` | 移除三个与端口冲突的空异常处理函数 |
| `User/App/Interface/app_freertos.h` | 新建启动接口 |
| `User/App/Src/app_freertos.c` | 新建两个验证任务和启动函数 |
| `User/App/Src/freertos_hooks.c` | 新建断言、内存失败和栈溢出处理 |
| `Core/Src/main.c` | 外设初始化后调用启动接口 |
| `Core/Src/dma.c`、USART6 NVIC 配置 | 后续接入中断与任务通信时调整优先级 |
| `base_control.ioc` | 同步时基、IRQ 优先级及代码生成设置，防止重新生成覆盖 |

`User/App/Interface` 和 `User/App/Src` 不存在时先创建。根 CMake 已包含 `User/App/Interface`，并自动收集 `User/` 中的新源文件。

## 3. 把正确的内核文件加入 CMake

这里采用显式列出源文件的方式。保留完整内核目录也可以，但不要递归把 `MiddleWares/FreeRTOS/portable/` 下所有 `.c` 加入构建，否则会混入其他处理器、其他编译器端口以及多个 heap 实现。

在根 `CMakeLists.txt` 中已有的 `add_executable(...)` 之后加入下面的块，例如放在收集 `USER_SOURCES` 的代码后面：

```cmake
set(FREERTOS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/MiddleWares/FreeRTOS")

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/event_groups.c
    ${FREERTOS_DIR}/stream_buffer.c
    ${FREERTOS_DIR}/timers.c
    ${FREERTOS_DIR}/portable/GCC/ARM_CM4F/port.c
    ${FREERTOS_DIR}/portable/MemMang/heap_4.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/portable/GCC/ARM_CM4F
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
)
```

`queue.c` 同时提供队列、信号量和互斥量的基础实现。`timers.c` 可以保留在源文件列表中；下面的初始配置关闭软件定时器，因此不会创建定时器服务任务。

本方案不再对内核目录调用 `add_subdirectory(MiddleWares/FreeRTOS)`。内核自带 CMake 集成也是可用方案，但不要与上面的手动源文件方案同时使用，以免重复编译。

确认编译内核的参数包含：

```text
-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
```

现有 `cmake/gcc-arm-none-eabi.cmake` 已指定 Cortex-M4、FPU 和 hard-float，建议把 `-mthumb` 也明确列入 `TARGET_FLAGS`。应用、HAL、内核和链接阶段必须使用一致的浮点 ABI。此端口自行管理浮点上下文，不需要额外添加其他端口的 `configENABLE_FPU` 配置。

## 4. 新建 FreeRTOSConfig.h

在 `Core/Inc/FreeRTOSConfig.h` 中放入下面的起步配置。32 KiB heap、任务优先级数量和栈大小是验证用预算，后续根据测量调整。

```c
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
#define configSUPPORT_STATIC_ALLOCATION          0
#define configTOTAL_HEAP_SIZE                    (32U * 1024U)
#define configAPPLICATION_ALLOCATED_HEAP         0

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
```

配置时注意：

- V11.3.1 中，`configTICK_TYPE_WIDTH_IN_BITS` 与旧的 `configUSE_16_BIT_TICKS` 只能定义一个。这里使用前者，且值是 `TICK_TYPE_WIDTH_32_BITS`，不要写成裸数字 `32`。
- 本方案不定义 `configSYSTICK_CLOCK_HZ`。该本地端口在未定义时使用核心时钟；盲目把它定义成 `SystemCoreClock` 会改变 SysTick 时钟源选择。
- `configCHECK_HANDLER_INSTALLATION = 1` 要配套有效的 `configASSERT`。本地端口检查 SVC 和 PendSV 向量，不检查 SysTick，因此仍需自行确认 SysTick 入口。
- 不需要把 `stm32f4xx_hal_conf.h` 中的 `USE_RTOS` 改为 1。它不是 FreeRTOS 启用开关，也不会自动让 HAL 驱动具备并发保护。
- `configSUPPORT_STATIC_ALLOCATION = 0` 时，当前方案不需要实现 Idle/Timer 任务的静态内存回调。
- 如果后续启用软件定时器，另行配置 `configTIMER_TASK_PRIORITY`、`configTIMER_QUEUE_LENGTH`、`configTIMER_TASK_STACK_DEPTH`；定时器回调不能执行阻塞等待。

配置项可对照 [FreeRTOS 官方配置说明](https://www.freertos.org/Documentation/02-Kernel/03-Supported-devices/02-Customization)，实际支持情况以本地 V11.3.1 的 `include/FreeRTOS.h` 为准。

## 5. 接通 SVC、PendSV、SysTick，保留 TIM14

采用第 4 步的直接映射后，`port.c` 会定义这三个符号：

```text
SVC_Handler      -> 启动第一个任务等端口处理
PendSV_Handler   -> 任务上下文切换
SysTick_Handler  -> FreeRTOS tick
```

需要从 `Core/Src/stm32f4xx_it.c` 中移除以下三个已有的空函数定义，包含函数体一起移除：

```c
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
```

上面列出的是待定位的函数签名，不是要求添加三个新函数。`Core/Inc/stm32f4xx_it.h` 中的函数声明可以保留；启动汇编中的向量项和弱别名也保留。

不要保留同名空实现，也不要用普通 C 函数包装调用 `vPortSVCHandler()` 或 `xPortPendSVHandler()`；该端口的异常处理包含专用汇编，需要按端口要求进入。

保持现有 HAL 时基路径：

```text
TIM14 -> TIM8_TRG_COM_TIM14_IRQHandler
      -> HAL_TIM_IRQHandler(&htim14)
      -> HAL_TIM_PeriodElapsedCallback
      -> HAL_IncTick()

SysTick -> FreeRTOS 端口 -> 内核 tick / 调度
```

当前 `main.c` 的 `HAL_TIM_PeriodElapsedCallback()` 已在 `htim->Instance == TIM14` 时调用 `HAL_IncTick()`，应保留。不要再在 SysTick 中增加一次 `HAL_IncTick()`，否则 HAL 毫秒计数会被重复推进。

调度器启动时，端口会配置 SysTick，并设置 PendSV/SysTick 为最低优先级；不要再独立调用 `HAL_SYSTICK_Config()` 重配它。本地 V11.3.1 的 CM4F 端口自行设置这些优先级，配置文件里的惯常 `configKERNEL_INTERRUPT_PRIORITY` 定义并不是该端口此处的执行来源。

CubeMX 重新生成代码后必须复查这三个函数是否又被生成。在 CubeMX 的 NVIC 代码生成设置中禁止生成冲突的处理函数；具体 UI 可因版本而异，最终以生成文件和链接符号为准。

## 6. 配置中断优先级

保持 `NVIC_PRIORITYGROUP_4`，使用全部 4 位作为抢占优先级。当前 `.ioc` 已使用该分组；后续不要在其他初始化代码中改成带子优先级的分组。

采用本配置时：

| HAL/CMSIS 中设置的 IRQ 优先级数值 | 是否允许调用相应 FreeRTOS `FromISR` API |
|---:|---|
| 0～4 | 不允许 |
| 5～15 | 允许，但仍必须使用中断专用 API |

硬件 IRQ 的数值越小，抢占能力越高；FreeRTOS 任务优先级则是数值越大越优先。这是两套不同的优先级，不要混用。

`configMAX_SYSCALL_INTERRUPT_PRIORITY` 是移位后的 `0x50`；传给 `HAL_NVIC_SetPriority()` 的则是未移位的 `5`，不能把 `0x50` 传给 HAL。原理参见 [FreeRTOS Cortex-M 中断优先级说明](https://www.freertos.org/Documentation/02-Kernel/03-Supported-devices/04-Demos/ARM-Cortex/RTOS-Cortex-M3-M4)。

当前 `Core/Src/dma.c` 中是：

```c
HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
```

若 DMA 回调将通知任务、发送队列或释放信号量，需要在启用该业务前改为例如：

```c
HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
```

若后续使用 USART6 的 IDLE 接收回调，并在回调中调用 RTOS API，还需要启用 USART6 全局中断、提供 `USART6_IRQHandler()` 调用 `HAL_UART_IRQHandler(&huart6)`，并将 USART6 IRQ 也设为 5～15。仅启用 DMA IRQ 不等于已经配置 UART IDLE IRQ。

不要仅因加入 RTOS 就盲目改动所有中断。TIM14 当前只推进 HAL tick，可以保留原优先级；不要在 ISR、关中断区域或临界区里调用 `HAL_Delay()` 或其他依赖中断推进的阻塞等待。

## 7. 实现错误处理 Hook

新建 `User/App/Src/freertos_hooks.c`。这些 Hook 在第 4 步已经启用，因此必须提供对应定义。

```c
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"

/* 可以在调试器中查看，也可以在 vAssertCalled 上打断点。 */
const char * volatile g_rtos_assert_file;
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
```

这里故意只记录信息并停机供调试，不在故障 Hook 中执行串口打印、申请内存或阻塞等待。栈溢出检测有助于发现问题，但不能替代合理的栈预算。

## 8. 创建两个最小验证任务

新建 `User/App/Interface/app_freertos.h`：

```c
#ifndef APP_FREERTOS_H
#define APP_FREERTOS_H

void appFreeRTOSStart(void);

#endif
```

新建 `User/App/Src/app_freertos.c`：

```c
#include "app_freertos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_led.h"
#include "main.h"

static void redLedTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        ledToggle(RED);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void greenLedTask(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        ledToggle(GREEN);
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(200));
    }
}

void appFreeRTOSStart(void)
{
    BaseType_t result;

    result = xTaskCreate(redLedTask, "red_led", 256,
                         NULL, 1, NULL);
    if (result != pdPASS)
        Error_Handler();

    result = xTaskCreate(greenLedTask, "green_led", 256,
                         NULL, 2, NULL);
    if (result != pdPASS)
        Error_Handler();

    vTaskStartScheduler();

    /* 正常启动后不返回；若返回，检查内存及端口启动条件。 */
    Error_Handler();
}
```

两个任务都必须无限循环，并通过阻塞 API 让出 CPU。任务入口不能直接返回；需要结束的任务使用 `vTaskDelete(NULL)`。

这里的栈深度 `256` 单位是 `StackType_t` 元素，在这个 CM4F 端口中为 32 位，因此对应 1024 字节，不是 256 字节。红灯每 500 ms 翻转一次，完整亮灭周期约 1 s；绿灯每 200 ms 翻转一次，完整周期约 400 ms。

`vTaskDelay()` 是相对延时，`xTaskDelayUntil()` 适合固定周期任务。任务执行超时后，后者不会自动弥补业务的计算能力，需要另外监测任务是否错过周期。

## 9. 在 main 中启动调度器

在 `main.c` 的 `USER CODE BEGIN Includes` 中增加：

```c
#include "app_freertos.h"
```

保留硬件初始化顺序：

```c
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
MX_DMA_Init();
MX_USART6_UART_Init();
```

在 `USER CODE BEGIN 2` 中调用：

```c
appFreeRTOSStart();
```

首次验证先不要在它前面执行 `imuInit(&imu)`。两个验证任务调度成功后，再把 IMU 的初始化和持续读取安排到专门的任务中。

正常启动后，程序的业务运行在任务里，`main()` 后面的裸机 `while (1)` 不再承担业务工作。不要把需要运行的初始化或循环放到 `appFreeRTOSStart()` 后面。

## 10. 编译、检查符号，再上板

在工程根目录执行：

```sh
cmake --preset Debug
cmake --build --preset Debug
```

工程已有相应的 Configure/Build Preset。重新配置是为了让新增内核源文件和头文件路径进入构建；构建输出位于 `build/Debug/`。

检查编译命令中是否包含所选端口和唯一 heap：

```sh
rg 'ARM_CM4F/port.c|MemMang/heap_4.c|FreeRTOS/tasks.c' \
    build/Debug/compile_commands.json
```

确认最终 ELF 中存在所需异常入口和调度器符号：

```sh
arm-none-eabi-nm -n build/Debug/base_control.elf | \
    rg ' (SVC_Handler|PendSV_Handler|SysTick_Handler|vTaskStartScheduler)$'
```

正常情况下三个入口都来自 `port.c` 的强定义。`nm` 可确认符号存在，具体归属再检查链接生成的 `base_control.map`；保留了一个空 SysTick 也可能有同名符号，不能只靠“名字存在”判断接线正确。

上板验证顺序：

1. 在 `appFreeRTOSStart()`、两个任务入口、`vAssertCalled()` 上打断点。
2. 确认两个 `xTaskCreate()` 都成功。
3. 确认调用 `vTaskStartScheduler()` 后进入任务。
4. 继续运行，观察两个 LED 按不同周期翻转。
5. 确认 `xTaskGetTickCount()` 在运行中增长；`vTaskDelay()` 后任务能够再次运行。
6. 确认 `HAL_GetTick()` 也继续增长，证明 TIM14 的 HAL 时基没有丢失。
7. 观察一段时间，确认没有进入内存失败、栈溢出或断言 Hook。

调试器单步和断点会改变时间观察结果，检查闪烁周期时应让程序连续运行。

## 11. 确认 heap 和任务栈预算

本项目链接脚本定义了普通 RAM 192 KiB 和 CCMRAM 64 KiB。`heap_4.c` 默认的 `ucHeap` 数组位于 `.bss`，按现有链接脚本进入普通 RAM；不能把两个内存区简单合并后当作一个自动可用的 FreeRTOS heap。

- `configTOTAL_HEAP_SIZE`：FreeRTOS heap 池，任务栈、TCB、动态队列等会消耗它。
- `xTaskCreate()` 的栈深度：每个任务自己的栈，按 32 位 word 计数。
- 链接脚本的 `_Min_Stack_Size`：启动与异常上下文使用的主栈预算，不是所有任务栈的总和。
- 链接脚本的 `_Min_Heap_Size`：C 运行库的堆预留，不会替代 `configTOTAL_HEAP_SIZE`。

运行后用 `xPortGetFreeHeapSize()`、`xPortGetMinimumEverFreeHeapSize()` 观察 heap；用 `uxTaskGetStackHighWaterMark()` 观察任务栈的历史最小剩余量，返回值在这里也是 word。

后续 IMU 解包的局部数组、浮点计算、库函数调用都会增加任务栈开销。初始 256-word LED 栈不能直接作为 IMU/控制任务的最终栈配置。启用 `configUSE_NEWLIB_REENTRANT` 也会改变内存预算，且不等于所有外设打印操作自动线程安全。

关于不同 heap 实现的用途，可参考 [FreeRTOS 内存管理说明](https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/09-Memory-management/01-Memory-management)。本方案只使用 `heap_4.c`，不要同时加入其他 `heap_*.c`。

## 12. 基础调度成功后再接入 Saber/IMU

推荐按以下顺序继续：

1. 新建一个 IMU 任务，让该任务独占 Saber 所用串口的接收和初始化流程。
2. 先用已验证的接收及解析方案获取完整 `imuData_t`，通过队列或受保护的缓存交给控制任务。
3. 阻塞式 HAL 接收仍然会占用当前任务的 CPU 时间，它不是 RTOS 的“等待事件后让出 CPU”。不要让高优先级任务长期在 HAL 轮询里等待串口。
4. 接入 DMA 后，让 ISR/回调记录接收进度并通知 IMU 任务，拼帧、校验和解包放在任务中完成。
5. 在开始 DMA 接收之前创建好被通知任务、队列或信号量，避免回调使用尚未初始化的句柄。
6. 使用持久有效的 DMA 缓冲区，明确生产者和消费者的覆盖边界；通知本身并不会防止循环 DMA 覆盖尚未处理的数据。

中断通知任务的典型形式如下。这里的 `imu_task_handle` 是后续应用需要保存的任务句柄，下面只是接口用法，不属于前面 LED 示例的必需代码：

```c
BaseType_t higher_priority_task_woken = pdFALSE;

vTaskNotifyGiveFromISR(imu_task_handle, &higher_priority_task_woken);
portYIELD_FROM_ISR(higher_priority_task_woken);
```

任务侧可用 `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)` 等待事件。ISR 使用 `...FromISR` 版本，任务使用普通版本；ISR 不能等待互斥量，也不能调用 `vTaskDelay()`。中断优先级必须符合第 6 步。

控制任务如果需要固定周期，可使用 `xTaskDelayUntil()`；若周期或抖动要求比 RTOS tick 更严格，需要另行设计硬件定时器触发方案，不能仅提高任务优先级就认为实时性已经得到保证。

## 13. 常见故障定位

| 现象 | 优先检查 |
|---|---|
| 找不到 `FreeRTOSConfig.h` | 配置文件位置及该目标的 include 路径 |
| 找不到 `portmacro.h` | 是否只加入了 `portable/GCC/ARM_CM4F` 的头文件路径 |
| `port.c` 提示需要硬件浮点 | 内核编译命令是否继承 FPU/ABI 参数，端口是否选错 |
| `pvPortMalloc` 等重复定义 | 是否编译了多个 heap，或同时采用两种内核 CMake 集成方式 |
| SVC/PendSV/SysTick 重复定义 | 是否仍保留 `stm32f4xx_it.c` 中的三个空实现 |
| 缺少 `vAssertCalled` 或 Hook | 第 7 步源文件是否被加入构建、名字和签名是否一致 |
| 调度器启动时进入断言 | 查看记录的文件/行号；检查实际 VTOR 向量、端口、IRQ 优先级配置 |
| 任务只运行一次，延时后不醒 | SysTick 是否仍为空处理函数，tick 是否增长，中断是否一直被关闭 |
| `HAL_GetTick()` 不增长 | TIM14 初始化、IRQ 和 `HAL_TIM_PeriodElapsedCallback()` 是否保留 |
| `xTaskCreate()` 失败或进入内存 Hook | heap 是否足够，是否错误理解了栈深度单位 |
| 接入 DMA 通知后断言或崩溃 | DMA/UART 的 IRQ 优先级、`FromISR` 用法、任务句柄及缓冲区生命周期 |
| 低优先级任务一直不运行 | 高优先级任务是否没有阻塞，或持续进行 HAL 轮询 |
| 重新生成 CubeMX 代码后失效 | 三个异常处理函数是否重新出现，优先级和时基是否被覆盖 |

## 14. 完成标准与验证范围

满足以下条件，才算完成本工程的基础内核移植：

- [ ] 只构建一个正确端口和一个 heap 实现。
- [ ] `FreeRTOSConfig.h` 与本地 V11.3.1 匹配，编译链接通过。
- [ ] SVC、PendSV、SysTick 进入 FreeRTOS 端口处理函数。
- [ ] HAL 的 TIM14 时基继续工作。
- [ ] 两个 LED 任务能够持续按各自周期运行。
- [ ] heap 与栈有测量依据，没有进入错误 Hook。
- [ ] 需要使用 RTOS API 的 IRQ 已设置允许的优先级。
- [ ] 再生 CubeMX 代码后，上述配置仍保持一致。

本指南依据当前仓库的构建文件、启动文件、HAL 时基和本地内核源码编写。交付前在项目外的临时目录完成了以下检查：

- 从本文原样提取 `FreeRTOSConfig.h`、应用头文件、任务示例和 Hook 示例。
- 使用实际 `arm-none-eabi-gcc`，以 Cortex-M4F/hard-float 参数和 `-Wall -Wextra -Werror` 编译第 3 步列出的 8 个内核源文件及两个应用示例源文件，全部通过。
- 在临时副本中移除三个空异常处理函数，编译剩余中断文件，并与上述对象文件进行可重定位链接，未发现重复定义。
- 检查 `port.c` 生成的对象文件，确认它提供 `SVC_Handler`、`PendSV_Handler`、`SysTick_Handler` 三个强符号。

这些检查验证了示例配置、源文件选择和符号接入方式；可重定位链接允许保留外部未解析符号，不等于最终固件已经完整链接或能够上板运行。本次没有构建最终固件、烧录设备或修改项目源码。上板时序、任务栈余量和 IMU 并发行为仍需要按前述步骤在实际硬件上验证。
