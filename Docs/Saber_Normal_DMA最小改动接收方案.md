# Saber 使用 Normal DMA 的最小改动接收方案

本文解决的目标是：减少 `HAL_UART_Receive()` 等待串口数据时的 CPU 占用，同时尽量保留原来的收帧、校验和数据解析代码。

推荐采用 **Normal DMA + IDLE 接收 + FreeRTOS StreamBuffer**。DMA 负责搬运字节，任务通过 StreamBuffer 阻塞等待数据，不需要自己维护环形 DMA 的读写位置。

本文是实现指导，示例用于说明接口与调用顺序，不包含完整的错误恢复实现。

## 1. 当前项目情况与改动范围

按 2026-09-29 的源码：

- USART6 RX DMA 已设置为 `DMA_NORMAL`，串口为 115200、8N1。
- `saberInit()` 配置 Saber 为 100 Hz，然后启动 `HAL_UARTEx_ReceiveToIdle_DMA()`。
- `saberRead()` 调用 `saberParseDataFrame()`，收帧底层仍有 `HAL_UART_Receive()`。
- `imuTask()` 每 10 ms 调用一次读取接口，并将成功解析的数据覆盖到最新状态队列。
- CMake 已编译 `stream_buffer.c`，FreeRTOS 已开启任务通知及动态、静态内存分配。

**当前需要解决的混用问题是：DMA 启动后，原来的收帧函数不能继续调用 `HAL_UART_Receive()`。** 两种接收方式共用同一个 UART RX 状态，不能同时使用。

建议按下面的范围修改：

| 位置 | 改动 |
|---|---|
| CubeMX USART6 RX DMA | 确认 Normal 模式，并保留 UART、DMA 中断 |
| `saber.c` | 创建 StreamBuffer，增加接收回调与字节读取包装函数 |
| `saberRecvFrame()` | 替换三处底层接收调用，保留找帧头、读长度、帧尾和 BCC 校验 |
| `saberInit()` / `imuInit()` | 明确初始化与 DMA 接收阶段，设备初始化成功后再创建读取任务 |
| `imuTask()` | 数据读取改为阻塞等待后，去掉固定 10 ms 延时 |
| `imuRead()` 与最新状态队列 | 继续提供最新解析状态，无需改为读取原始 DMA 缓冲 |

源码参考：[Saber 驱动](../User/Devices/Src/saber.c)、[串口配置](../Core/Src/usart.c)、[IMU 任务](../User/Modules/Src/imu.c)、[FreeRTOS 配置](../Core/Inc/FreeRTOSConfig.h)、[CMake](../CMakeLists.txt)。

## 2. 为什么这样可以释放 CPU

`HAL_UART_Receive()` 会轮询 UART 状态，直到收够指定字节数或超时。等待期间，调用任务仍在执行。

换成 DMA 后，字节搬运由硬件完成；换成 StreamBuffer 阻塞读取后，没有数据时，接收任务进入阻塞态，调度器可以运行其他任务。

```text
Saber UART
    → Normal DMA 缓冲区
    → IDLE / 接收满回调
    → StreamBuffer，复制有效字节
    → saberReceiveBytes，任务阻塞读取
    → saberRecvFrame，组帧与校验
    → saberParseDataFrame，解析物理量
    → 最新状态队列
    → 其他任务通过 imuRead 获取状态
```

启动 DMA 后继续用 `while` 检查标志、比较首尾字节，仍然属于忙等。释放 CPU 的关键是使用 FreeRTOS 的阻塞等待机制。

StreamBuffer 内部也使用软件缓存，但读写位置由 FreeRTOS 管理。应用只调用收发接口，不需要手动拼接整个 DMA 数组。

## 3. 缓冲区与接收启动

### 3.1 两个缓冲区承担不同职责

| 缓冲区 | 用途 | 初步选择 |
|---|---|---|
| DMA 缓冲区 | 接收一批原始字节，在 IDLE 或接收满后交给软件 | 保留现有 `dma_buf[256]` |
| StreamBuffer | 保存等待任务消费的字节流，连接不同接收块 | 当前 100 Hz 可先使用 512 字节 |

这些状态应放在 `saber.c` 内部，不向其他模块暴露原始数组。512 字节只是当前速率下的起点，容量仍应覆盖任务最长无法处理数据的时间。

首次初始化时可以创建 StreamBuffer：

```c
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"

static StreamBufferHandle_t saber_rx_stream;
static bool saber_dma_mode = false;
static volatile bool saber_rx_fault = false;

// 首次初始化中执行；重复初始化时不要重复分配
if (saber_rx_stream == NULL)
    saber_rx_stream = xStreamBufferCreate(512, 1);

if (saber_rx_stream == NULL)
    return false;
```

第二个参数 `1` 表示有至少一个字节时可以唤醒等待的接收任务。若希望保持静态分配风格，也可以使用 `xStreamBufferCreateStatic()`，按项目所用版本的参数要求准备持久存储。

这里的 `saber_dma_mode` 表示“底层接收使用 StreamBuffer”，不是“DMA 当前一定在运行”。DMA 故障时应设置错误状态并恢复接收，不能通过把该标志清零而自动退回轮询接收。

### 3.2 每次启动都检查返回值、关闭 HT

Normal DMA 接收在 IDLE 或接收满时结束，随后需要重新启动：

```c
static bool saberStartReceive(saberCtx_t *saber)
{
    if (!saber || !saber->huart || !saber->huart->hdmarx)
        return false;

    if (HAL_UARTEx_ReceiveToIdle_DMA(saber->huart,
                                   dma_buf,
                                   sizeof(dma_buf)) != HAL_OK)
        return false;

    __HAL_DMA_DISABLE_IT(saber->huart->hdmarx, DMA_IT_HT);
    return true;
}
```

半传输事件 HT 发生时，DMA 尚未结束，不能把它当作一次完整接收后重新启动。这里关闭 HT，只处理 IDLE 与接收满 TC。HAL 启动新的 DMA 接收会重新配置中断，因此每次启动后都要关闭 HT。

## 4. 接收回调：复制字节后立即重启

使用 `HAL_UARTEx_RxEventCallback()` 接收 IDLE / TC 事件。该回调是中断上下文，处理顺序为：

1. 确认是 Saber 使用的 UART，并确认事件为 IDLE 或 TC。
2. 检查 `Size` 有效且不超过 DMA 缓冲区容量。
3. 将 `dma_buf[0..Size)` 复制到 StreamBuffer。
4. 检查实际写入长度；不足表示发生字节丢失，需要通知解析任务重新同步。
5. 立即重新启动 Normal DMA，并检查启动结果。
6. 接收恢复后，再按需要请求任务切换。

向 StreamBuffer 写入的关键调用为：

```c
BaseType_t higher_priority_task_woken = pdFALSE;

size_t copied = xStreamBufferSendFromISR(
    saber_rx_stream,
    dma_buf,
    Size,
    &higher_priority_task_woken);

// copied != Size：记录丢失，标记接收流不连续
// 无论缓存是否有余量，都应处理下一次DMA启动
// 启动失败：设置接收故障标志，由任务协调恢复

portYIELD_FROM_ISR(higher_priority_task_woken);
```

上面只展示写入 API；实际回调要补齐事件过滤、重启调用和错误状态处理。`portYIELD_FROM_ISR()` 应放在重新启动接收之后。

必须复制字节，不能只保存 `dma_buf` 的指针：重新启动后，DMA 会覆盖这块内存。回调也不应执行浮点解析、`printf` 或阻塞等待。

**IDLE 事件表示线路空闲，不等于 Saber 协议帧结束。** 一帧可以分到多个接收块，一块也可以包含多帧。将有效字节依次放进 StreamBuffer 后，原来的收帧函数仍可按协议识别边界。

### 中断与单生产者要求

StreamBuffer 按单生产者、单消费者使用。本方案中，UART IDLE 和 DMA TC 都可能进入回调，应确保两路写入串行执行。

当前 DMA2 Stream1 中断优先级为 5，USART6 为 13，FreeRTOS 的中断调用阈值为 5；这两个优先级都允许调用相应的 `FromISR` API。为了避免两路回调写入时相互抢占，可以在 CubeMX 中将它们设置为相同抢占优先级，例如 5；也可以用适当的中断临界区保护生产操作。

相关背景见 [FreeRTOS 任务与中断优先级配置说明](FreeRTOS_任务与中断优先级配置说明.md)。不要因为使用了 `FromISR` API，就假定多个生产者可以自动并发写入 StreamBuffer。

## 5. 字节读取包装：保留原来的收帧逻辑

### 5.1 包装函数需要保持的语义

`HAL_UART_Receive()` 返回成功时，表示请求的字节数已收齐。

`xStreamBufferReceive()` 返回的是本次实际读取的字节数，可能少于请求长度。因此包装函数必须累计读取，收够指定长度才返回 `HAL_OK`；没有数据时阻塞等待，并使用统一的剩余超时时间。

以下为适用于现有有限毫秒超时参数的示例。错误状态的设置与恢复由回调和接收任务配合完成。将辅助函数放在调用者之前，或在文件现有静态函数声明处补充原型。

```c
static HAL_StatusTypeDef saberReceiveBytes(saberCtx_t *saber,
                                          uint8_t *dst,
                                          uint16_t size,
                                          uint32_t timeout_ms)
{
    if (!saber || !saber->huart || !dst || size == 0)
        return HAL_ERROR;

    // 初始化配置期间，尚未启动DMA，沿用原接收方式
    if (!saber_dma_mode)
        return HAL_UART_Receive(saber->huart, dst, size, timeout_ms);

    // 本示例使用有限超时，不提供无限等待的错误唤醒机制
    if (!saber_rx_stream || timeout_ms == HAL_MAX_DELAY)
        return HAL_ERROR;

    size_t received = 0;
    TimeOut_t time_state;
    TickType_t remaining = pdMS_TO_TICKS(timeout_ms);

    if (timeout_ms > 0 && remaining == 0)
        remaining = 1;

    vTaskSetTimeOutState(&time_state);

    for (;;)
    {
        if (saber_rx_fault)
            return HAL_ERROR;

        received += xStreamBufferReceive(saber_rx_stream,
                                         dst + received,
                                         size - received,
                                         remaining);

        if (saber_rx_fault)
            return HAL_ERROR;

        if (received == size)
            return HAL_OK;

        if (xTaskCheckForTimeOut(&time_state, &remaining) != pdFALSE)
            return HAL_TIMEOUT;
    }
}
```

超时或故障时，可能已经消费了部分字节。调用者应丢弃本次不完整帧，随后重新寻找帧头，不能把 `dst` 当作完整有效数据使用。

### 5.2 替换 saberRecvFrame 的三处接收调用

原来的调用：

```c
HAL_UART_Receive(saber->huart, dst, size, timeout);
```

改成：

```c
saberReceiveBytes(saber, dst, size, timeout);
```

分别替换：找帧头的单字节接收、读取剩余头部的 4 字节接收、读取 Payload 加 BCC 和帧尾的接收。现有 `!= HAL_OK` 判断仍可沿用。

`saberRecvFrame()` 的帧头识别、容量检查、帧尾判断和 BCC 校验，以及上层 PID、浮点数解析可以继续使用。

包装函数的总超时只约束一次指定长度的读取。若还需要限制整帧接收或整个 ACK 握手的总时间，应在外层使用统一截止时间，不能每找到一个字节就重新开始整帧计时。

## 6. 初始化与任务运行阶段如何衔接

为了少改代码，可以先保留初始化期间的阻塞通信，只将持续测量数据的接收改为 DMA：

```text
systemTask 调用 imuInit
    → saberInit：唤醒、进入配置、设置数据包与频率
    → 保持 saber_dma_mode = false，配置ACK仍走原接收方式
    → 创建 StreamBuffer，准备回调所需的设备上下文
    → 进入 MEASURE，完成最后一次配置ACK接收
    → 选择DMA接收后端，启动Normal DMA
    → 启动成功，设备初始化返回成功
    → imuInit 创建 IMU 任务
    → imuTask 消费 StreamBuffer 并解析数据
```

这部分初始化发生在 `systemTask` 中，不能当作“调度器启动前”的代码处理。这里只是保留一次性的初始化忙等；持续运行时通过 StreamBuffer 阻塞等待。

当前 `imuInit()` 在调用设备初始化前就创建了 IMU 任务。推荐保留最新状态队列的创建位置，将 IMU 任务的分配、创建语句移到 `imu->ops->init(imu->ctx)` 成功之后；整个初始化成功后再设置模块的初始化成功标志。

这样，读取任务开始运行时，StreamBuffer、设备上下文和接收后端都已准备好，`saberRead()` 可以保持当前形式：

```c
static bool saberRead(void *ctx, imuData_t *data)
{
    saberCtx_t *saber = ctx;
    return saberParseDataFrame(saber, data);
}
```

如果必须提前创建读取任务，应让它先通过任务通知或事件等待初始化完成，再开始消费接收数据。普通 `bool`，或者只添加 `volatile`，不能替代任务间的同步发布机制。

DMA 接收后端启用后，配置 ACK 或测量数据的接收都应通过同一个字节入口。多个任务不能同时消费同一个 StreamBuffer。以后需要运行中重新配置时，应先协调停止测量消费，再由配置任务独占接收流程。

## 7. IMU 任务改为由数据到达驱动

读取接口已经能够阻塞等待新数据，任务可以去掉原来的固定 10 ms 延时：

```c
static void imuTask(void *arg)
{
    imu_t *imu = arg;

    for (;;)
    {
        imuData_t sample = {0};

        if (imu->ops->read(imu->ctx, &sample))
            xQueueOverwrite(sampleQueue, &sample);
        else
            vTaskDelay(1);
    }
}
```

健康接收时，数据未到达就阻塞，收到有效帧后发布状态，再继续等待。失败后的短延时避免故障状态下反复返回而形成忙循环；实际恢复由接收任务协调完成。

其他任务仍通过 `imuRead()` / `xQueuePeek()` 获取最新状态。若一次积压了多帧，可以逐帧校验后只发布最后一帧，减少旧状态对外暴露的时间。

## 8. Normal 模式的边界与错误恢复

当前 57 字节帧、115200、8N1 下：

```text
发送一帧约需 57 × 10 / 115200 ≈ 4.95 ms
100 Hz 的帧周期为 10 ms
若帧连续发送，帧间约有 5.05 ms 空闲
```

这个速率适合作为 Normal DMA 方案的起点。但结束接收到重新启动之间存在空窗，仍需保证中断响应、复制和重启及时；对于无间隙的连续高速字节流，Circular DMA 通常更合适。

实际实现需要明确处理以下情况：

- **StreamBuffer 写入不足**：记录字节丢失，标记流不连续，让当前收帧返回失败。由消费任务协调清理残留和组帧状态，再重新寻找帧头。
- **DMA 重启失败**：设置故障标志，任务检查 UART / DMA 状态后恢复接收，不能把未启动的 DMA 当作正常运行。
- **UART 或 DMA 错误**：配合 HAL 的错误回调及中止完成状态处理，由任务执行恢复；有限接收超时也能让等待任务返回并检查故障。
- **接收恢复**：先协调停止旧接收、排除旧回调，再重置软件缓存和帧状态，成功重启后才恢复正常读取。不要在任意中断中直接清空一个仍被任务使用的 StreamBuffer。

本方案减少的是持续运行期间的轮询与手工缓冲管理，协议中的长度、BCC 和 PID 校验仍然需要保留。

进一步提高输出频率时，带宽与容量选择见 [Saber 串口带宽与波特率配置说明](Saber_串口带宽与波特率配置说明.md)。
