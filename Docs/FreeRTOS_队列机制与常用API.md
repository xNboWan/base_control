# FreeRTOS 队列机制与常用 API

本文基于本项目的 FreeRTOS V11.3.1，使用原生 FreeRTOS API。先解释队列如何传递数据和调度任务，再介绍常用函数，最后结合 Saber IMU 给出使用方案。

文中的代码是参考示例，不表示已经接入项目。FreeRTOS 的构建、任务创建和调度器启动见 [FreeRTOS 移植指南](FreeRTOS_移植指南.md)。

## 1. 队列解决什么问题

当一个任务产生数据，另一个任务需要处理这些数据时，可以通过队列交接。例如：

```text
Saber 接收解析任务  ──发送 imuData_t──>  队列  ──接收 imuData_t──>  控制任务
```

队列同时承担两件事：

- **保存数据**：接收方暂时没有运行时，数据可以先留在队列中。
- **协调任务运行**：队列为空时，接收任务可以等待；新数据到来时，内核使等待的任务进入就绪态。

一个队列允许多个发送者和多个接收者。内核保护队列内部操作，通常不需要在每次发送、接收外面再套一个互斥量。但多个接收者是在竞争数据：一条消息被 `xQueueReceive()` 成功取走后，其他接收者就收不到这一条。**普通队列不提供广播。**

## 2. 队列如何保存数据

### 2.1 创建时确定容量和元素大小

```c
#include "FreeRTOS.h"
#include "queue.h"
#include "imu.h"

QueueHandle_t queue = xQueueCreate(8, sizeof(imuData_t));
```

这表示最多保存 **8 个 `imuData_t`**，不是 8 个字节。创建后，容量和每个元素的大小固定，不会随发送次数自动扩容。

本项目的 `imuData_t` 包含三个加速度、三个角速度和一个偏航角。在当前 STM32 GCC ABI 下，通常是 28 字节，代码中仍应使用 `sizeof(imuData_t)`。8 个元素的数据存储区约为 224 字节，此外还有队列控制结构、分配器开销及对齐开销。

队列内部可理解为一块循环使用的存储区，加上读写位置、当前元素数量，以及等待发送和等待接收的任务列表。

### 2.2 默认先进先出

使用 `xQueueSend()` 或 `xQueueSendToBack()` 时，新数据放到队尾；使用 `xQueueReceive()` 时，从队头取出最早的数据。

```text
发送 A、B、C 后：  队头 [ A ][ B ][ C ] 队尾
接收一次得到 A：  队头 [ B ][ C ] 队尾
```

`xQueueSendToFront()` 可以插入队头，但会改变正常的处理顺序。反复插入队头可能使原有数据长时间得不到处理。

任务优先级决定哪些任务先运行，不会自动把队列中的消息按照“重要程度”重新排序。

### 2.3 队列按值复制

```c
imuData_t sample = {0};
sample.yaw = 30.0f;

if (xQueueSend(queue, &sample, 0) == pdPASS)
{
    sample.yaw = 60.0f;
}
```

成功发送时，内核把 `sizeof(imuData_t)` 个字节复制进队列。因此，这次入队的数据中 `yaw` 仍然是 `30.0f`。发送返回后，发送者可以复用原来的变量。

接收时也会复制：内核把队列中的数据复制到接收方提供的变量中，再移除该队列元素。接收变量必须有足够的空间。

`QueueHandle_t` 不记录供编译器检查的消息类型。创建时的 `sizeof(...)` 和收发变量不匹配，可能编译通过却产生越界或错误数据。

注意：队列保护的是队列内部状态，不会阻止其他任务或 DMA 在复制期间修改发送源缓冲区。应先得到稳定、完整的数据，再发送。官方的[队列概述](https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/02-Queues-mutexes-and-semaphores/01-Queues)也说明了这种复制语义。

## 3. 队列的“阻塞”等待是否占用 CPU

**任务因为等待队列而处于 Blocked 状态时，不执行代码，不占用 CPU 执行时间。** 内核可以调度其他就绪任务，没有业务任务就绪时运行空闲任务。

例如：

```c
imuData_t sample;

if (xQueueReceive(queue, &sample, pdMS_TO_TICKS(20)) == pdPASS)
{
    /* 收到了一个采样。 */
}
else
{
    /* 等待期限内未能取得数据。 */
}
```

如果队列已经有数据，立即取出；如果为空，任务最多按设置的 tick 超时等待。在等待期间收到数据，不需要等满 20 ms 才处理。

这与反复检查串口标志的轮询等待不同。之前 `saberRead()` 中调用的轮询式 `HAL_UART_Receive()` 会在等待标志时执行循环；队列等待则让任务交出 CPU。

### 3.1 `xTicksToWait` 的含义

| 参数 | 行为 |
|---|---|
| `0` | 不等待条件满足；队列满或空时立即返回失败 |
| `pdMS_TO_TICKS(20)` | 满或空时，按换算后的 tick 数等待 |
| `portMAX_DELAY` | 配合 `INCLUDE_vTaskSuspend == 1`，不因队列等待超时而返回 |

本项目 `configTICK_RATE_HZ == 1000`，一个 tick 是 1 ms；`INCLUDE_vTaskSuspend == 1`，支持上述无限期等待。仍建议用 `pdMS_TO_TICKS()` 表达毫秒，避免以后修改 tick 频率时出错。

超时以 tick 为单位，并受 tick 相位影响；任务恢复就绪后还要等待调度，所以不能把它当成精确的微秒计时或函数返回时间上限。较低 tick 频率下，过短的毫秒值还可能换算为 0。

`0` 表示不等待队列条件，并不表示函数完全没有执行成本，也不禁止正常的任务切换。

### 3.2 哪个等待任务先被唤醒

有多个任务等待同一个队列时，优先级较高的等待任务优先被唤醒；相同优先级时，等待较久的优先。发送使接收者就绪，接收腾出空间则可能使发送者就绪。

“就绪”不等于“已经执行”。本项目开启抢占调度，唤醒更高优先级任务后可能立即发生切换，发送者可能在接收者处理完后才继续运行。

### 3.3 避免把非阻塞 API 写成忙等

下面会反复检查队列、消耗 CPU：

```c
while (xQueueReceive(queue, &sample, 0) != pdPASS)
{
    /* 一直轮询。 */
}
```

专门等待数据的任务应使用带等待时间的接收。周期控制任务则可以每周期尝试读取一次，没有新数据时按照自己的超时和控制策略处理，然后等待下一个周期。

不要在 ISR、临界区或调度器暂停期间执行可能阻塞的队列操作。调度器启动前可以创建队列；带等待的操作应放在已经运行的任务中。

## 4. 常用 API 速查

先包含 `FreeRTOS.h`，再包含 `queue.h`；任务示例另外需要 `task.h`。下面省略内部属性修饰和宏展开，按日常调用形式展示。

| API | 用途 | 成功结果 / 失败结果 |
|---|---|---|
| `xQueueCreate(length, itemSize)` | 动态创建 | 队列句柄 / `NULL` |
| `xQueueCreateStatic(length, itemSize, storage, control)` | 静态创建 | 有效参数下返回句柄 |
| `xQueueSend(q, &item, ticks)` | 发送到队尾 | `pdPASS` / `errQUEUE_FULL` |
| `xQueueSendToBack(q, &item, ticks)` | 与 `xQueueSend()` 等价 | `pdPASS` / `errQUEUE_FULL` |
| `xQueueSendToFront(q, &item, ticks)` | 插入队头 | `pdPASS` / `errQUEUE_FULL` |
| `xQueueReceive(q, &item, ticks)` | 取出并移除队头元素 | `pdPASS` / `errQUEUE_EMPTY` |
| `xQueuePeek(q, &item, ticks)` | 复制队头元素，保留在队列中 | `pdPASS` / `errQUEUE_EMPTY` |
| `xQueueOverwrite(q, &item)` | 覆盖长度为 1 的队列 | 合法调用返回 `pdPASS` |
| `uxQueueMessagesWaiting(q)` | 查询当前元素个数 | 数量，类型 `UBaseType_t` |
| `uxQueueSpacesAvailable(q)` | 查询当前空位个数 | 数量，类型 `UBaseType_t` |
| `xQueueReset(q)` | 清空队列，保留队列对象 | 有效队列正常返回 `pdPASS` |
| `vQueueDelete(q)` | 删除队列对象 | 无返回值 |

这些接口不负责让非法句柄变得安全。创建成功、参数有效和生命周期正确都是调用前提。

### 4.1 `xQueueCreate()`：动态创建

```c
QueueHandle_t xQueueCreate(UBaseType_t uxQueueLength,
                           UBaseType_t uxItemSize);
```

`uxQueueLength` 是元素数量，普通数据队列的 `uxItemSize` 是单个元素的字节数。内存来自 FreeRTOS 堆。

```c
QueueHandle_t queue = xQueueCreate(8, sizeof(imuData_t));
if (queue == NULL)
{
    /* 创建失败：停止依赖该队列的启动流程，执行错误处理。 */
}
```

本项目已开启 `configSUPPORT_DYNAMIC_ALLOCATION`，堆预算为 32 KiB，由任务栈、任务控制块、队列等共同使用。不要在每次采样时创建队列；通常在初始化阶段创建一次。

### 4.2 `xQueueCreateStatic()`：应用提供内存

```c
QueueHandle_t xQueueCreateStatic(UBaseType_t uxQueueLength,
                                 UBaseType_t uxItemSize,
                                 uint8_t *pucQueueStorage,
                                 StaticQueue_t *pxQueueBuffer);
```

例如，下面两块内存都保持静态生命周期：

```c
static StaticQueue_t queueControl;
static uint8_t queueStorage[8 * sizeof(imuData_t)];

/* 在初始化函数中调用。 */
QueueHandle_t queue = xQueueCreateStatic(
    8, sizeof(imuData_t), queueStorage, &queueControl);
```

`queueStorage` 保存元素，`queueControl` 保存控制信息。两者在队列使用期间都必须有效，不能用一个即将返回的函数中的普通局部数组代替。

本项目当前 `configSUPPORT_STATIC_ALLOCATION == 0`，因此此示例需要先开启该配置。开启后还需提供 `vApplicationGetIdleTaskMemory()`；本版本的非 MPU 端口也可以通过 `configKERNEL_PROVIDED_STATIC_MEMORY == 1` 使用内核提供的实现。如果以后开启软件定时器，还要处理定时器任务的静态内存支持。静态队列不会消耗 FreeRTOS 动态堆，但仍占用 RAM。

### 4.3 `xQueueSend()`：发送数据

```c
BaseType_t xQueueSend(QueueHandle_t xQueue,
                      const void *pvItemToQueue,
                      TickType_t xTicksToWait);
```

第二个参数是待复制数据的地址。队列有空位就复制数据；队列满时按第三个参数决定立即失败还是等待。

```c
if (xQueueSend(queue, &sample, pdMS_TO_TICKS(5)) != pdPASS)
{
    /* 本次没有成功入队：记录丢样、上报错误或执行明确的重试策略。 */
}
```

成功表示已经放入队列，不表示接收方已经处理完成。如果发送方必须知道处理结果，需要另外设计确认消息。

### 4.4 `xQueueReceive()` 与 `xQueuePeek()`：消费还是查看

```c
BaseType_t xQueueReceive(QueueHandle_t xQueue,
                         void *pvBuffer,
                         TickType_t xTicksToWait);

BaseType_t xQueuePeek(QueueHandle_t xQueue,
                      void *pvBuffer,
                      TickType_t xTicksToWait);
```

两者都把一个元素复制到 `pvBuffer`。区别是 `Receive` 会移除这个元素，`Peek` 会保留它。[官方 Peek 说明](https://freertos.org/Documentation/02-Kernel/04-API-references/06-Queues/13-xQueuePeek)也明确指出，后续读取可以再次得到该元素。

对于一个装有 A、B 的队列，连续两次 `Peek` 都得到 A；先 `Receive` 得到 A，再 `Receive` 得到 B。

仅在返回成功时处理接收变量。返回失败不代表接收到了全零数据，变量也可能仍保留上一次的内容。

### 4.5 `xQueueOverwrite()`：只保留最新值

```c
BaseType_t xQueueOverwrite(QueueHandle_t xQueue,
                           const void *pvItemToQueue);
```

**只用于长度为 1 的队列。** 队列为空时写入，已有数据时覆盖旧值，不等待空位。

```c
QueueHandle_t latestQueue = xQueueCreate(1, sizeof(imuData_t));
/* 必须先确认 latestQueue != NULL。 */
xQueueOverwrite(latestQueue, &sample);
```

假设先后写入 A、B、C，期间没有人读取，最终只保留 C。适合“控制任务关心最新状态”的场景，不适合“每条记录都必须处理”的日志场景。

长度为 1 的普通 `xQueueSend()` 不会自动覆盖旧值；队列满时仍会等待或失败。

### 4.6 查询、清空和删除

`uxQueueMessagesWaiting()` 和 `uxQueueSpacesAvailable()` 返回某一时刻的状态。其他任务或中断可能随后改变队列，因此不要把“先查询有空位，再发送”当成一定成功的保证；直接调用收发 API 并检查结果。

`xQueueReset()` 丢弃尚未接收的所有元素，保留内存和句柄。等待接收的任务不会因此获得数据；等待发送的任务可能因出现空位而被唤醒。它不是让所有等待任务退出的“取消”接口。

`vQueueDelete()` 删除对象；动态队列对应的内存被释放，静态队列的应用自有内存不会交给堆释放。删除前，必须确保没有任务或 ISR 继续访问它，也没有任务还阻塞在它上面。删除后旧句柄失效。

如果元素里存的是指针，清空或删除队列都不会自动释放指针指向的对象。

## 5. 在中断中使用队列

ISR 不能等待队列条件，需要使用专门的 `FromISR` 接口。普通 `xQueueSend(..., 0)` 即使等待时间为 0，也不能代替 ISR 版本。

| ISR API | 行为 |
|---|---|
| `xQueueSendFromISR(q, &item, &woken)` | 发送到队尾，满则返回 `errQUEUE_FULL` |
| `xQueueSendToBackFromISR(q, &item, &woken)` | 与上一项等价 |
| `xQueueSendToFrontFromISR(q, &item, &woken)` | 插入队头，满则失败 |
| `xQueueOverwriteFromISR(q, &item, &woken)` | 覆盖长度为 1 的队列 |
| `xQueueReceiveFromISR(q, &item, &woken)` | 取走一个元素，空则返回 `pdFAIL` |
| `xQueuePeekFromISR(q, &item)` | 查看队头，空则返回 `pdFAIL` |
| `uxQueueMessagesWaitingFromISR(q)` | 查询当前元素个数 |
| `xQueueIsQueueEmptyFromISR(q)` | 空时返回 `pdTRUE` |
| `xQueueIsQueueFullFromISR(q)` | 满时返回 `pdTRUE` |

ISR 收发成功均可用 `== pdPASS` 判断。查询仍然只是瞬时状态。没有常规的 `xQueueResetFromISR()`、`vQueueDeleteFromISR()` 或 `uxQueueSpacesAvailableFromISR()` 可供使用。

下面演示一个由 ISR 调用的短事件发布函数。事件队列应提前用 `xQueueCreate(8, sizeof(uint32_t))` 创建并检查成功，再启用事件来源。

```c
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdint.h>

/* 示例：由初始化代码赋值，只供这个事件来源使用。 */
static QueueHandle_t eventQueue;
static volatile uint32_t droppedEvents;

void publishEventFromISR(uint32_t event)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    if (xQueueSendFromISR(eventQueue, &event,
                         &higherPriorityTaskWoken) != pdPASS)
    {
        ++droppedEvents;
    }

    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}
```

`higherPriorityTaskWoken` 与发送是否成功是两回事：API 返回值表示是否入队；这个输出参数表示此次操作是否唤醒了比被中断任务优先级更高的任务。为 `pdTRUE` 时，`portYIELD_FROM_ISR()` 请求在中断退出后进行调度。在 Cortex-M4F 端口中，这通过请求 PendSV 完成。

如果在同一次 ISR 内连续调用多个相关 API，可以共用一个初始化为 `pdFALSE` 的标志，最后统一请求切换；不要在每个调用之间把已经置位的标志清零。

### 本工程的中断优先级要求

当前配置是：

```c
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY     15
```

配合当前 NVIC 分组，调用 FreeRTOS `FromISR` API 的外设中断，其 HAL 抢占优先级数值应在 **5～15**。数值 0～4 的中断紧急程度更高，不能调用这些 API。任务优先级则是数值越大越优先，两者不要混淆。

编写本文时，`Core/Src/dma.c` 中 DMA2 Stream1 优先级仍为 0。若在该 DMA ISR 调用链里的 HAL 回调中使用队列或任务通知，需要先调整，例如：

```c
HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
```

UART IDLE 中断若调用同类 API，也应满足这一要求。实际执行上下文决定用普通接口还是 `FromISR` 接口，不能仅凭函数名称中有没有“Callback”判断。

## 6. 结合 Saber：用长度为 1 的队列传递最新采样

建议把串口接收、协议解析和上层取数分开：

```text
UART → DMA 接收缓冲区 → 接收任务组帧、校验、解析
                                  ↓
                      xQueueOverwrite(最新采样队列)
                                  ↓
                       saberRead() → 控制任务
```

队列中放的是已经校验和解析完成的 `imuData_t`，不是尚未收齐的一段串口字节。DMA 缓冲区、接收位置跟踪、帧解析和接收任务通知仍需单独实现。

### 6.1 上下文需要保存队列句柄

以下是对现有 `saberCtx_t` 的扩展示意：

```c
#include "FreeRTOS.h"
#include "queue.h"

typedef struct
{
    UART_HandleTypeDef *huart;
    QueueHandle_t sample_queue;
    /* DMA 缓冲区、解析状态等按接收方案另行添加。 */
} saberCtx_t;
```

这样每个 Saber 实例可以持有自己的数据队列，统一的 `imu_t` 和 `imuData_t` 不需要知道 FreeRTOS 队列的细节。

### 6.2 初始化、发布和读取示例

下面的三个函数以“头文件已经增加 `sample_queue` 字段”为前提，可作为驱动实现的参考。创建函数只调用一次，并在启动接收任务及其数据来源前完成；失败时不继续启动接收链路。

```c
#include "saber.h"
#include "FreeRTOS.h"
#include "queue.h"

bool saberSampleQueueCreate(saberCtx_t *saber)
{
    if (saber == NULL)
        return false;

    saber->sample_queue = xQueueCreate(1, sizeof(imuData_t));
    return saber->sample_queue != NULL;
}

/* 仅在接收任务中调用：sample 已经完成整帧校验和字段解析。 */
bool saberPublishSample(saberCtx_t *saber, const imuData_t *sample)
{
    if (saber == NULL || sample == NULL || saber->sample_queue == NULL)
        return false;

    return xQueueOverwrite(saber->sample_queue, sample) == pdPASS;
}

static bool saberRead(void *ctx, imuData_t *data)
{
    saberCtx_t *saber = ctx;

    if (saber == NULL || data == NULL || saber->sample_queue == NULL)
        return false;

    return xQueueReceive(saber->sample_queue, data, 0) == pdPASS;
}
```

通过已有的 `.read = saberRead` 绑定，上层仍然调用 `imuRead(&imu, &data)`。这里 `true` 表示取到一个尚未被消费的采样，`false` 表示没有新采样或参数无效。

`saberRead()` 不等待串口，也不启动 DMA。它只复制队列中的结果。周期控制任务每周期调用一次即可；如果循环中持续尝试读取而不延时，仍会形成忙等。

这个方案适合一个控制任务消费最新状态。慢消费者会跳过部分中间采样，这是覆盖策略的预期行为。需要每帧积分、离线分析或连续记录时，应使用足够容量的 FIFO，并设计溢出检测和处理，不能直接套用“只保留最新值”。

### 6.3 改用 `Peek` 会发生什么

如果把最后一行改成：

```c
return xQueuePeek(saber->sample_queue, data, 0) == pdPASS;
```

读取不会清空队列。第一次采样写入后，即使设备停止更新，后续调用也可能一直成功并返回旧数据。因此它的含义是“存在一个缓存值”，不是“本次获得了新数据”。

如果需要多个任务读取同一份最新状态，可以约定都使用 `Peek`，并为消息增加时间戳或序号，让每个任务自行判断是否更新和是否过期。这也不保证每个任务都看到了每一次更新；严格广播每条消息需要给各接收者独立队列或另行设计发布机制。

### 6.4 队列与任务通知如何配合

对于 DMA 接收，可以在中断里用任务通知表达“有接收进展”，由接收任务读取 DMA 中新增字节；解析完成后再用队列传递结构体。通知负责唤醒，队列负责交接采样值。

仅有通知次数并不能还原字节流。循环 DMA 仍需正确跟踪读写位置、处理回绕和覆盖风险。DMA 原始缓冲区要长期有效，并位于 STM32F427 的 DMA 可访问 SRAM 中。

## 7. 传结构体和传指针的区别

对于本项目这种小型采样结构体，直接按值发送最容易管理。

若消息很大，也可以创建保存指针的队列：

```c
QueueHandle_t pointerQueue = xQueueCreate(4, sizeof(imuData_t *));
/* 以下演示一次交接；实际使用前必须检查 pointerQueue != NULL。 */
static imuData_t storedSample;
imuData_t *sendPtr = &storedSample;
if (xQueueSend(pointerQueue, &sendPtr, 0) == pdPASS)
{
    /* 接收方使用完成前，不修改 storedSample。 */
}

/* 接收方： */
imuData_t *receivePtr = NULL;
if (xQueueReceive(pointerQueue, &receivePtr, 0) == pdPASS)
{
    /* 通过 receivePtr 访问对象，处理完成后按约定归还所有权。 */
}
```

这里展示参数形式，不包含完整的缓冲区归还协议。队列复制的是指针值，所以发送参数是 `&sendPtr`，不是 `sendPtr`。它不会把指针指向的整个结构体复制进去。静态存储只能保证地址长期有效，不能允许发送者在接收方处理期间覆盖对象。

指针方案必须约定所有权：

1. 发送成功后，发送者能否继续修改或复用这块内存？通常应等接收者处理完再归还。
2. 发送失败时，对象仍由谁负责回收？通常仍由发送者负责。
3. 接收完成后，谁释放对象或归还内存池？

不能把即将失效的栈变量地址放入队列后就返回，也不能把循环 DMA 正在覆盖的区域当成稳定消息直接交给慢消费者。普通队列不会解决这些生命周期和并发访问问题。

## 8. 怎样选择队列长度和通信机制

| 需求 | 适合的起点 |
|---|---|
| 控制任务只关心最新采样 | 长度 1，发送用 `xQueueOverwrite()` |
| 顺序处理每条命令或采样 | 多元素 FIFO，检查发送失败并定义溢出策略 |
| ISR 只需唤醒一个指定任务 | 任务通知，数据缓冲另行管理 |
| 传递连续字节流 | 根据读写者模型考虑流缓冲区或 DMA 环形缓冲区 |
| 保护共享外设的独占访问 | 互斥量；普通数据队列不提供互斥量的优先级继承 |

FIFO 长度应覆盖消费者短暂无法运行时的积压。例如采样率 200 Hz，消费者可能连续 30 ms 不能处理，仅这段停顿就可能积压约 6 条，还应考虑已有积压、相位和突发裕量。长期平均处理速度低于产生速度时，再大的有限队列最终也会满。

队列操作会复制数据并执行同步和调度逻辑。传递很大的元素会增加复制和临界区成本，尤其不适合在高频 ISR 中频繁进行。可以考虑稳定缓冲区加描述符，但需要承担上一节的所有权管理。

## 9. 常见问题排查

| 现象 | 优先检查 |
|---|---|
| `xQueueCreate()` 返回 `NULL` | 堆是否足够，是否反复创建，元素大小和数量是否写反 |
| 接收到的数据错乱 | 队列元素大小与变量是否匹配，是否误把指针当结构体，发送源是否同时被修改 |
| 发送经常失败 | 消费者是否运行、优先级和处理速度是否合理，队列是否已满 |
| `saberRead()` 经常返回 `false` | 是否根本没有解析出新帧，是否读得比采样快，是否还有其他消费者取走数据 |
| 读到数据但设备其实已经掉线 | 是否用 `Peek` 反复读取旧值，是否缺少时间戳及超时判断 |
| 空队列等待仍占用很多 CPU | 是否在循环里反复调用超时为 0 的接收 |
| ISR 调用时触发断言或系统异常 | 是否用 `FromISR` 版本，NVIC 优先级是否允许调用 FreeRTOS |
| 使用覆盖接口触发断言 | 队列长度是否确实为 1 |
| 多个任务收不到同一条消息 | 是否把普通队列误当成广播通道 |
| 删除或重新初始化后异常 | 是否还有旧句柄、等待任务或中断在访问对象 |

## 10. 源码与参考资料

本文 API 参数、返回值和实现行为主要依据项目内核源码核对：

- [queue.h](../MiddleWares/FreeRTOS/include/queue.h)：公开接口、参数注释和使用示例。
- [queue.c](../MiddleWares/FreeRTOS/queue.c)：循环存储、数据复制、等待列表、覆盖和复位逻辑。
- [tasks.c](../MiddleWares/FreeRTOS/tasks.c)：任务等待、超时和调度。
- [FreeRTOSConfig.h](../Core/Inc/FreeRTOSConfig.h)：本项目 tick、内存和中断优先级配置。
- [官方队列概述](https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/02-Queues-mutexes-and-semaphores/01-Queues)与[官方队列头文件](https://github.com/FreeRTOS/FreeRTOS-Kernel/blob/main/include/queue.h)：进一步阅读；在线主分支可能与本项目版本不同，使用时以本地版本为准。
