# FreeRTOS 任务与中断优先级配置说明

本文针对 `base_control` 工程，记录截至 **2026-09-29** 的优先级配置，并说明后续接入 IMU、遥控、电机回传和 UART8 日志时的设置原则。工程使用 STM32F427、Cortex-M4F 和 FreeRTOS 原生 API。

当前最需要关注的两点：

- **UART8 中断优先级为 14，可以在中断回调中使用适用的 `…FromISR()` API。**
- **USART6 和 DMA2_Stream1 中断优先级仍为 0；如果后续通过它们通知任务，需要先调整优先级。**

文中的建议值供后续修改参考，与当前已经生效的配置分别列出。

## 1. 任务优先级与中断优先级是两套规则

| 比较项 | FreeRTOS 任务优先级 | NVIC 中断优先级 |
|---|---|---|
| 当前有效范围 | 0～7 | 0～15 |
| 数字方向 | 数字越大，优先级越高 | 数字越小，优先级越高 |
| 示例 | 任务 3 优先于任务 1 | 中断 5 优先于中断 14 |
| 主要作用 | 决定哪个就绪任务先运行 | 决定中断的抢占顺序，以及能否调用内核 API |

两套数字不能直接比较。例如，不能因为任务优先级是 3、中断优先级是 14，就认为该任务能抢占该中断。中断是否执行由中断使能、屏蔽状态和异常优先级决定。

当前 [FreeRTOSConfig.h](../Core/Inc/FreeRTOSConfig.h) 中的相关配置为：

```c
#define configUSE_PREEMPTION                     1
#define configUSE_TIME_SLICING                   1
#define configTICK_RATE_HZ                       1000U
#define configMAX_PRIORITIES                     8

#define configPRIO_BITS                          4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY  15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
```

`configMAX_PRIORITIES = 8` 表示有 8 个任务优先级，即 **0～7**，不包含 8。任务优先级规则见 [FreeRTOS 官方说明](https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/01-Tasks-and-co-routines/03-Task-priorities)。

## 2. 当前任务优先级

| 任务 | 当前优先级 | 当前行为 | 配置位置 |
|---|---:|---|---|
| SYSTEM | 3 | 初始化 IMU、DEBUG 等模块，随后进入长延时 | [system.c](../User/Modules/Src/system.c) 的 `systemLaunch()` |
| IMU | 1 | 接收、解析惯导数据，更新长度为 1 的最新状态队列 | [imu.c](../User/Modules/Src/imu.c) 的 `imuInit()` |
| DEBUG | 1 | 每 50 ms 读取一次 IMU 状态 | [debug.c](../User/Modules/Src/debug.c) 的 `debugInit()` |
| Idle | 0 | 没有其他就绪任务时运行 | FreeRTOS 自动创建 |

当前 `configUSE_TIMERS = 0`，没有软件定时器服务任务。UART8 专用日志发送任务如果后续加入，需要在其创建位置单独设置任务优先级；UART8 的中断优先级 14 不等于发送任务的优先级。

### 2.1 抢占与时间片的实际含义

当前开启抢占和时间片：

- 高优先级任务一旦进入就绪状态，可以抢占低优先级任务。
- 同优先级的就绪任务分享时间片，所以 IMU 和 DEBUG 可以轮流运行。
- 高优先级任务一直处于就绪状态时，低优先级任务不能靠时间片获得运行机会。

当前 tick 频率为 1000 Hz，即一个 tick 为 1 ms。这不表示每个任务都能每 1 ms 执行一次，也不表示任务的运行周期一定是 1 ms。[FreeRTOS 调度说明](https://github.com/FreeRTOS/FreeRTOS-Kernel-Book/blob/main/ch04.md#4123-prioritized-preemptive-scheduling-with-time-slicing)

### 2.2 当前不宜直接提高 IMU 优先级

[saber.c](../User/Devices/Src/saber.c) 的 `saberRecvFrame()` 当前使用 `HAL_UART_Receive()` 轮询接收。HAL 所说的“阻塞接收”表示函数等待接收完成，但等待期间仍在执行轮询，**没有使调用任务进入 FreeRTOS 的阻塞状态**。

同时，当前 `imuTask()` 成功读取数据后立即继续下一次读取，只有失败分支调用 `vTaskDelay(1)`。

如果直接把 IMU 从 1 提高到 2 或 3，IMU 在持续轮询时可能长期占用 CPU，使 DEBUG=1 和 Idle 得不到运行。即使调用 `taskYIELD()`，只要 IMU 仍然是最高优先级的就绪任务，也不能保证 DEBUG 得到运行机会。

建议分两步处理：

1. 当前轮询阶段暂时保持 **SYSTEM=3、IMU=1、DEBUG=1**。
2. 改成“中断／DMA 接收数据 → 通知 IMU 任务 → IMU 任务解析并发布最新状态”。IMU 无数据时阻塞等待通知，完成后再次等待；此时可以考虑 **IMU=2、DEBUG／日志发送=1**。

上述任务优先级是本项目的起步建议，不是 FreeRTOS 的固定要求。后续控制任务应根据响应期限和实际执行时间确定优先级，不应只按模块名称分配。

### 2.3 初始化顺序不能依赖优先级

当前 `imuInit()` 先创建 IMU 任务，再调用 `imu->ops->init()` 初始化设备。现在 SYSTEM=3、IMU=1，通常由 SYSTEM 继续执行初始化，但调整优先级，或初始化过程进入 RTOS 阻塞等待后，IMU 可能在设备尚未准备好时运行。

更稳妥的顺序是：

```text
创建最新状态队列 → 初始化设备并检查结果 → 创建 IMU 任务
```

如果必须先创建任务，就让 IMU 任务启动后先阻塞，收到“初始化完成”通知后再开始接收。

## 3. 当前中断与异常优先级

当前 [base_control.ioc](../base_control.ioc) 使用 `NVIC_PRIORITYGROUP_4`：4 位全部用于抢占优先级，没有子优先级位，因此 `HAL_NVIC_SetPriority()` 的第三个参数填写 `0`。[ST 官方定义](https://github.com/STMicroelectronics/stm32f4xx-hal-driver/blob/master/Inc/stm32f4xx_hal_cortex.h#L82)

| 中断／异常 | 当前优先级 | 设置建议或说明 |
|---|---:|---|
| UART8 | 14 | 可以保留，适合日志发送；允许调用适用的 `…FromISR()` |
| USART6 | 0 | 当前不允许调用 FreeRTOS API；接入中断通知后可设为 5 |
| DMA2_Stream1 | 0 | 当前不允许调用 FreeRTOS API；接入 DMA 回调通知后可设为 5 |
| TIM8_TRG_COM_TIM14 | 15 | 当前用于 TIM14 HAL 时基，可以保留 |
| SysTick | 15 | FreeRTOS tick，由移植层管理 |
| PendSV | 调度器启动后为 15 | 用于任务切换；不要按 `.ioc` 中的 0 判断运行时优先级 |
| SVC／SVCall | 调度器启动后为 0 | 用于启动首个任务，保持移植层配置 |
| MemManage／MemoryManagement | 0 | 当前无需调整 |
| BusFault | 0 | 当前无需调整 |
| UsageFault | 0 | 当前无需调整 |
| DebugMonitor | 0 | 当前无需调整 |
| NMI、HardFault | 硬件固定高优先级 | 不属于普通的 0～15 可配置优先级 |

外设中断设置位于 [usart.c](../Core/Src/usart.c) 和 [dma.c](../Core/Src/dma.c)。HAL tick 优先级来自 [stm32f4xx_hal_conf.h](../Core/Inc/stm32f4xx_hal_conf.h) 中的 `TICK_INT_PRIORITY = 15U`。

本项目的 [ARM_CM4F/port.c](../MiddleWares/FreeRTOS/portable/GCC/ARM_CM4F/port.c) 在 `xPortStartScheduler()` 中将 PendSV、SysTick 设置为最低优先级，将 SVCall 设置为最高可配置优先级。因此，不要把 SVC、PendSV、SysTick 三个异常统一改成 15；保留当前移植层的处理即可。NMI、HardFault 的固定优先级见 [Arm 官方说明](https://arm-software.github.io/CMSIS_6/v6.0.0/Core/group__NVIC__gr.html)。

## 4. 哪些中断可以调用 FreeRTOS

当前 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`。对于 UART、DMA 等应用中断，规则如下：

| 中断优先级 | 能否调用 FreeRTOS API | 原因 |
|---|---|---|
| 0～4 | **不能，包括 `…FromISR()`** | 优先级高于内核允许的中断调用范围 |
| 5～15 | 可以调用适用的 **`…FromISR()`** | 位于内核允许的中断调用范围内 |

这意味着：

- UART8=14 已满足要求，**不必一定设为 5**。
- USART6=0、DMA2_Stream1=0 本身并非错误；如果完全不调用 FreeRTOS API，可以保留。需要调用内核时，才必须调整。
- `HAL_UART_IRQHandler()` 本身不等于调用 FreeRTOS。需要检查它触发的回调，以及回调继续调用的函数。
- `HAL_UART_TxCpltCallback()`、UART 接收回调、DMA 回调若由中断触发，仍处于中断上下文，继承对应中断的限制。
- SVC、PendSV、SysTick 是内核内部异常，由移植层管理，不按应用外设中断的方式修改。

常见的中断通信接口包括 `xQueueSendFromISR()`、`xSemaphoreGiveFromISR()` 和 `vTaskNotifyGiveFromISR()`。需要切换到被唤醒的高优先级任务时，按照 API 用法检查 `xHigherPriorityTaskWoken` 并调用 `portYIELD_FROM_ISR()`。

中断中不能改用普通任务版 API，也不能调用 `vTaskDelay()`、等待互斥量或执行可能阻塞的 `printf()`。中断应完成必要的数据搬运和任务通知，把解析、格式化输出等工作放到任务中。[FreeRTOS Cortex-M 说明](https://freertos.org/Documentation/02-Kernel/03-Supported-devices/04-Demos/ARM-Cortex/RTOS-Cortex-M3-M4)

### 4.1 HAL 参数填写 5，不是 0x50

HAL 接受未移位的优先级数字。例如，下面是未来接入任务通知时的建议配置：

```c
HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
HAL_NVIC_SetPriority(UART8_IRQn, 14, 0);
```

FreeRTOS 的硬件宏则根据 4 位优先级进行移位：

```text
configMAX_SYSCALL_INTERRUPT_PRIORITY = 5  << 4 = 0x50
configKERNEL_INTERRUPT_PRIORITY      = 15 << 4 = 0xF0
```

不要把 `0x50` 直接填写到 HAL 的优先级参数中。[FreeRTOS Cortex-M 优先级说明](https://freertos.org/Documentation/02-Kernel/03-Supported-devices/04-Demos/ARM-Cortex/RTOS-Cortex-M3-M4)

### 4.2 临界区不能屏蔽所有中断

本项目的 Cortex-M4F 移植层使用 BASEPRI 保护内核临界区。当前阈值为 5 时，`taskENTER_CRITICAL()` 会屏蔽优先级 **5～15** 的中断，而 **0～4** 仍可执行。

因此，如果任务和优先级 0～4 的中断共同访问一份数据，仅使用 `taskENTER_CRITICAL()` 不能保证互斥。这也解释了为什么这些高优先级中断不能调用 FreeRTOS API：它们可能打断内核正在修改的数据结构。[FreeRTOS Cortex-M 临界区说明](https://freertos.org/Documentation/02-Kernel/03-Supported-devices/04-Demos/ARM-Cortex/RTOS-Cortex-M3-M4)

## 5. 后续接入时的操作顺序

1. **先保证初始化顺序**：队列、通知目标和设备状态准备好后，再允许相关任务和中断开始工作。
2. **改造接收等待方式**：让 IMU 任务无数据时阻塞等待通知，避免高优先级轮询占用 CPU。
3. **检查整个中断回调调用链**：凡是要调用 FreeRTOS 的应用中断，都设置在 5～15，并使用适用的 `…FromISR()` API。
4. **再调整任务优先级**：完成事件驱动接收后，可考虑 IMU=2，DEBUG 和 UART8 日志发送任务=1。
5. **在 CubeMX 中保存中断配置**：通过 `System Core → NVIC` 设置优先级，并保存 `.ioc`，避免只修改生成的 `usart.c`、`dma.c` 后又被重新生成覆盖。
6. **验证实际响应**：检查数据更新周期、接收丢帧、日志输出和任务是否长期得不到运行。优先级设置应服务于实际响应要求。

当前建议保留 UART8=14；USART6 和 DMA2_Stream1 是否改为 5，取决于后续回调是否调用 FreeRTOS。
