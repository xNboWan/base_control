# STM32 CAN HAL API 与使用示例

> 适用范围：本工程的 STM32F427IIH6、STM32F4 HAL **V1.8.5**，使用新版 `stm32f4xx_hal_can.h`（bxCAN，经典 CAN）。文中的代码是**接入示例**，不是本工程已有的 CAN 实现。

## 先看本工程的现状

本工程目前没有配置 CAN：`base_control.ioc` 的外设列表不含 CAN，`Core/Inc/stm32f4xx_hal_conf.h` 中的 `HAL_CAN_MODULE_ENABLED` 被注释，本地没有 `stm32f4xx_hal_can.h/.c`，CMake 也没有编译 CAN 驱动。因此，直接复制下文代码会因缺少 `hcan1`、驱动等而无法编译。

接入 CAN1 时，先完成这些步骤：

1. 在 CubeMX 中启用 CAN1，按实际电路选择 CAN_RX/CAN_TX 引脚，配置位时序和所需中断；重新生成 `can.c/.h`、MSP 和中断入口。引脚要以板级原理图为准。
2. 从与本工程一致的 [ST STM32F4 HAL V1.8.5](https://github.com/STMicroelectronics/stm32f4xx-hal-driver/tree/v1.8.5) 取得 [`stm32f4xx_hal_can.h`](https://github.com/STMicroelectronics/stm32f4xx-hal-driver/blob/v1.8.5/Inc/stm32f4xx_hal_can.h) 和 [`stm32f4xx_hal_can.c`](https://github.com/STMicroelectronics/stm32f4xx-hal-driver/blob/v1.8.5/Src/stm32f4xx_hal_can.c)，放入对应 `Drivers/STM32F4xx_HAL_Driver/Inc`、`Src` 目录。
3. 在 `Core/Inc/stm32f4xx_hal_conf.h` 启用 `HAL_CAN_MODULE_ENABLED`；确认 `cmake/stm32cubemx/CMakeLists.txt` 将 `stm32f4xx_hal_can.c` 列入源文件。只启用宏而不加入源文件会在链接时缺少实现。
4. 实车总线还需要 CAN 收发器、CANH/CANL 接线及总线两端终端电阻；仅 MCU 的 CAN_TX/CAN_RX 引脚不能直接接 CANH/CANL。普通模式发送还需要其他节点提供 ACK；单板验证可先用 `CAN_MODE_LOOPBACK`。

本工程 `.ioc` 显示 APB1/PCLK1 为 **45 MHz**，下面的 500 kbit/s 位时序据此计算。改动时钟树后须重新计算。项目使用 FreeRTOS；如果 CAN 中断回调调用 `xQueueSendFromISR` 等 FreeRTOS API，中断抢占优先级须设置为数值 **5～15**（见 `Core/Inc/FreeRTOSConfig.h` 的 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`）。

## 一、基本调用顺序与数据类型

常见顺序为：

```text
配置时钟/GPIO/NVIC（HAL_CAN_MspInit 或 CubeMX 生成）
    → HAL_CAN_Init
    → HAL_CAN_ConfigFilter
    → HAL_CAN_Start
    → HAL_CAN_ActivateNotification（需要中断时）
    → HAL_CAN_AddTxMessage / HAL_CAN_GetRxMessage
```

`HAL_CAN_Init` 负责初始化控制器，**不会自动开始总线通信**；`HAL_CAN_Start` 后才进入监听/收发状态。过滤器也可以在运行中配置，但首次启动前先设置更直观。至少配置一个能接收目标帧的过滤器，否则可能看不到报文。

| 类型 | 用途 | 关键成员 |
| --- | --- | --- |
| `CAN_HandleTypeDef` | CAN 实例与运行状态 | `Instance`、`Init`、`State`、`ErrorCode` |
| `CAN_InitTypeDef` | 位时序和工作模式 | `Prescaler`、`Mode`、`SyncJumpWidth`、`TimeSeg1`、`TimeSeg2`、`TimeTriggeredMode`、`AutoBusOff`、`AutoWakeUp`、`AutoRetransmission`、`ReceiveFifoLocked`、`TransmitFifoPriority` |
| `CAN_FilterTypeDef` | 验收过滤器 | `FilterBank`、`FilterMode`、`FilterScale`、ID/掩码高低 16 位、`FilterFIFOAssignment`、`FilterActivation`、`SlaveStartFilterBank` |
| `CAN_TxHeaderTypeDef` | 发送帧头 | `StdId`、`ExtId`、`IDE`、`RTR`、`DLC`、`TransmitGlobalTime` |
| `CAN_RxHeaderTypeDef` | 接收帧头 | `StdId`、`ExtId`、`IDE`、`RTR`、`DLC`、`Timestamp`、`FilterMatchIndex` |

经典 CAN 标准 ID 为 11 位（`0～0x7FF`），扩展 ID 为 29 位（`0～0x1FFFFFFF`），每帧 `DLC` 为 `0～8`。`CAN_ID_STD` / `CAN_ID_EXT` 选择 ID 类型，`CAN_RTR_DATA` / `CAN_RTR_REMOTE` 选择数据帧 / 远程帧。本 HAL 是 bxCAN 接口，不是 FDCAN，也没有 CAN FD 的 64 字节数据段。

**缓冲区始终分配 8 字节。**此版本 `HAL_CAN_AddTxMessage` 在写邮箱时读取 `aData[0..7]`，`HAL_CAN_GetRxMessage` 在取帧时写入 `aData[0..7]`，即使 `DLC < 8` 也如此。发送和接收示例都使用 `uint8_t data[8]`，不可按 DLC 只分配 1～7 字节。

## 二、初始化、位时序与过滤器

### 1. `HAL_CAN_Init` / `HAL_CAN_DeInit` / MSP 回调

```c
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_DeInit(CAN_HandleTypeDef *hcan);
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan);
void HAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan);
```

首次 `HAL_CAN_Init` 会调用 `HAL_CAN_MspInit`，由用户或 CubeMX 生成代码使能 CAN/GPIO 时钟、配置引脚复用和 NVIC。`HAL_CAN_DeInit` 调用相应的 `MspDeInit`。在本版本中，这两个函数返回 `HAL_OK` 或 `HAL_ERROR`；初始化超时时返回 `HAL_ERROR`，并在 `HAL_CAN_GetError` 中记录 `HAL_CAN_ERROR_TIMEOUT`。

下面只展示 CubeMX 生成的 `MX_CAN1_Init()` 中与 HAL 有关的配置；`hcan1`、GPIO 和时钟配置必须先按“本工程现状”一节补齐：

```c
hcan1.Instance = CAN1;
hcan1.Init.Prescaler = 5;
hcan1.Init.Mode = CAN_MODE_NORMAL;           // 单板自测可改 CAN_MODE_LOOPBACK
hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
hcan1.Init.TimeSeg1 = CAN_BS1_14TQ;
hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
hcan1.Init.TimeTriggeredMode = DISABLE;
hcan1.Init.AutoBusOff = ENABLE;
hcan1.Init.AutoWakeUp = DISABLE;
hcan1.Init.AutoRetransmission = ENABLE;
hcan1.Init.ReceiveFifoLocked = DISABLE;
hcan1.Init.TransmitFifoPriority = DISABLE;

if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    Error_Handler();
}
```

位速率公式为 `PCLK1 / [Prescaler × (1 + BS1 + BS2)]`。本工程 PCLK1 为 45 MHz，上例得到 `45 MHz / [5 × (1 + 14 + 3)] = 500 kbit/s`，采样点为 `(1 + 14) / 18 ≈ 83.3%`。两端节点的位速率必须一致。`AutoRetransmission = ENABLE` 表示出错或未收到 ACK 时允许硬件重发；持续未收到 ACK 时，发送可能一直占用邮箱。

### 2. `HAL_CAN_ConfigFilter`

```c
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan,
                                       const CAN_FilterTypeDef *sFilterConfig);
```

过滤器决定哪些帧进入 FIFO0 或 FIFO1。下面的 32 位掩码过滤器**接收所有 ID 和帧类型**，适合先验证收发：

```c
CAN_FilterTypeDef filter = {0};
filter.FilterBank = 0;                        // CAN1 使用滤波器组 0
filter.FilterMode = CAN_FILTERMODE_IDMASK;
filter.FilterScale = CAN_FILTERSCALE_32BIT;
filter.FilterIdHigh = 0;
filter.FilterIdLow = 0;
filter.FilterMaskIdHigh = 0;
filter.FilterMaskIdLow = 0;                    // 全 0 掩码：不比较任何位
filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
filter.FilterActivation = CAN_FILTER_ENABLE;
filter.SlaveStartFilterBank = 14;              // 0～13 给 CAN1，14～27 给 CAN2

if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
    Error_Handler();
}
if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    Error_Handler();
}
```

只接收**标准数据帧 ID `0x123`**时，在调用 `HAL_CAN_ConfigFilter` 前保留其余字段，仅改成：

```c
filter.FilterIdHigh = 0x123U << 5;
filter.FilterIdLow = 0;
filter.FilterMaskIdHigh = 0x7FFU << 5;       // 比较标准 ID 的 11 位
filter.FilterMaskIdLow = 0x06U;              // 同时比较 IDE、RTR 位
```

如果控制器已经按“接收所有帧”的配置启动，改动 `filter` 变量后还须再次调用 `HAL_CAN_ConfigFilter(&hcan1, &filter)`，硬件过滤器才会更新。

`CAN_FILTERMODE_IDMASK` 的掩码位为 1 才比较对应位。STM32F427 的 CAN1/CAN2 共用 28 个过滤器组；`SlaveStartFilterBank = 14` 时前 14 组属于 CAN1，后 14 组属于 CAN2。若启用 CAN2，应为它选择相应过滤器组，并核对 CAN1/CAN2 时钟配置。

## 三、启动、停止与睡眠

```c
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_RequestSleep(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_WakeUp(CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_IsSleepActive(const CAN_HandleTypeDef *hcan);
```

| API | 作用与注意点 |
| --- | --- |
| `HAL_CAN_Start` | 在已初始化的 `READY` 状态启动，进入 `LISTENING` 状态。 |
| `HAL_CAN_Stop` | 从 `LISTENING` 状态停止，回到 `READY`。 |
| `HAL_CAN_RequestSleep` | 请求休眠；当前帧结束后才真正进入睡眠。 |
| `HAL_CAN_IsSleepActive` | 返回非 0 表示硬件已进入睡眠，不能只凭 `RequestSleep` 的成功返回值判断。 |
| `HAL_CAN_WakeUp` | 主动唤醒控制器；也可配置自动唤醒。 |

```c
if (HAL_CAN_RequestSleep(&hcan1) == HAL_OK) {
    uint32_t start = HAL_GetTick();
    while (HAL_CAN_IsSleepActive(&hcan1) == 0U) {
        if ((HAL_GetTick() - start) >= 10U) break; // 按系统需求调整超时
    }
}
(void)HAL_CAN_WakeUp(&hcan1);
```

## 四、发送 API 与示例

```c
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan,
                                       const CAN_TxHeaderTypeDef *pHeader,
                                       const uint8_t aData[], uint32_t *pTxMailbox);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan, uint32_t TxMailboxes);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef *hcan, uint32_t TxMailboxes);
uint32_t HAL_CAN_GetTxTimestamp(const CAN_HandleTypeDef *hcan, uint32_t TxMailbox);
```

控制器有 3 个发送邮箱。`HAL_CAN_AddTxMessage` 把帧写入空邮箱并请求发送，`HAL_OK` **只表示请求已入邮箱，不代表已在总线上成功发送或收到 ACK**。`pTxMailbox` 输出的是 `CAN_TX_MAILBOX0/1/2` 中的一个**位掩码**，可直接传给 `HAL_CAN_IsTxMessagePending` 或 `HAL_CAN_AbortTxRequest`；它不是数组下标。`GetTxTimestamp` 仅在启用时间触发通信模式时有意义。

发送标准数据帧：

```c
CAN_TxHeaderTypeDef tx = {0};
uint8_t data[8] = {0x11, 0x22, 0x33, 0x44, 0, 0, 0, 0};
uint32_t mailbox;

tx.StdId = 0x123;
tx.IDE = CAN_ID_STD;
tx.RTR = CAN_RTR_DATA;
tx.DLC = 4;                                   // 总线上只发送前 4 字节
tx.TransmitGlobalTime = DISABLE;

if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
    /* 三个邮箱均忙：稍后重试或放入应用发送队列 */
} else if (HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mailbox) != HAL_OK) {
    /* 查看 HAL_CAN_GetError(&hcan1) */
} else {
    /* mailbox 可用于查询待发送状态或取消本次请求 */
}
```

若需要确认发送成功，启用 `CAN_IT_TX_MAILBOX_EMPTY`、接通 `CAN1_TX_IRQHandler`，在对应的 `HAL_CAN_TxMailbox0/1/2CompleteCallback` 中处理。`HAL_CAN_IsTxMessagePending(...) == 0` 只表示不再待发送，不能单独证明发送成功（也可能已中止或出错）。

## 五、接收 API、轮询与中断示例

```c
uint32_t HAL_CAN_GetRxFifoFillLevel(const CAN_HandleTypeDef *hcan, uint32_t RxFifo);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t RxFifo,
                                       CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
```

`RxFifo` 取 `CAN_RX_FIFO0` 或 `CAN_RX_FIFO1`。`HAL_CAN_GetRxMessage` 取出一帧后会释放对应 FIFO 的一个槽位；`DLC` 说明本帧有效数据字节数，缓冲区仍须有 8 字节。

**轮询读取一次：**

```c
if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U) {
    CAN_RxHeaderTypeDef rx = {0};
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, data) == HAL_OK) {
        if (rx.IDE == CAN_ID_STD && rx.RTR == CAN_RTR_DATA && rx.StdId == 0x123) {
            /* 处理 data[0..rx.DLC-1]；DLC 最多为 8 */
        }
    }
}
```

**中断接收：**先完成过滤器配置并 `HAL_CAN_Start`，再激活通知；CubeMX 须使能 `CAN1_RX0_IRQn`，中断入口转给 HAL：

```c
if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
}

/* stm32f4xx_it.c 中；与 CubeMX 生成的同名入口合并，不要重复定义 */
void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
        CAN_RxHeaderTypeDef rx;
        uint8_t data[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, data) != HAL_OK) break;
        /* 将 rx 与 data 的内容复制到应用缓冲区/队列；不要保存局部变量指针。
           中断中避免打印、长时间解析或阻塞等待。 */
    }
}
```

收到消息的通知不会替应用读取 FIFO；回调既不读取报文、也不停用通知时，可能不断重新进入中断。如果用 FreeRTOS 队列传帧，队列元素应包含完整帧头及 `uint8_t data[8]`，回调中用 `xQueueSendFromISR`，并遵守前述中断优先级限制。

## 六、通知、中断入口与回调

```c
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t ActiveITs);
HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef *hcan, uint32_t InactiveITs);
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *hcan);
```

通知标志可按位或组合。常见组合及相应的 NVIC 入口：

| 通知标志 | 典型入口 | 对应回调 |
| --- | --- | --- |
| `CAN_IT_TX_MAILBOX_EMPTY` | `CAN1_TX_IRQHandler` | `HAL_CAN_TxMailbox0/1/2CompleteCallback`，或相应 `AbortCallback` |
| `CAN_IT_RX_FIFO0_MSG_PENDING`、`CAN_IT_RX_FIFO0_FULL` | `CAN1_RX0_IRQHandler` | `HAL_CAN_RxFifo0MsgPendingCallback`、`HAL_CAN_RxFifo0FullCallback` |
| `CAN_IT_RX_FIFO1_MSG_PENDING`、`CAN_IT_RX_FIFO1_FULL` | `CAN1_RX1_IRQHandler` | `HAL_CAN_RxFifo1MsgPendingCallback`、`HAL_CAN_RxFifo1FullCallback` |
| `CAN_IT_BUSOFF`、`CAN_IT_ERROR_WARNING`、`CAN_IT_ERROR_PASSIVE`、`CAN_IT_LAST_ERROR_CODE`，配合 `CAN_IT_ERROR` | `CAN1_SCE_IRQHandler` | `HAL_CAN_ErrorCallback` |
| `CAN_IT_SLEEP_ACK`、`CAN_IT_WAKEUP` | `CAN1_SCE_IRQHandler` | `HAL_CAN_SleepCallback`、`HAL_CAN_WakeUpFromRxMsgCallback` |

若要上报总线错误，示例为：

```c
(void)HAL_CAN_ActivateNotification(&hcan1, CAN_IT_BUSOFF | CAN_IT_ERROR);
/* CAN1_SCE_IRQHandler 中调用 HAL_CAN_IRQHandler(&hcan1) */
```

上述回调在 HAL 中都有弱定义，常用做法是在用户源文件中实现同名函数。全部事件回调的签名都是 `void 回调名(CAN_HandleTypeDef *hcan)`：

- 发送邮箱：`HAL_CAN_TxMailbox0CompleteCallback`、`HAL_CAN_TxMailbox1CompleteCallback`、`HAL_CAN_TxMailbox2CompleteCallback`；`HAL_CAN_TxMailbox0AbortCallback`、`HAL_CAN_TxMailbox1AbortCallback`、`HAL_CAN_TxMailbox2AbortCallback`。
- 接收 FIFO：`HAL_CAN_RxFifo0MsgPendingCallback`、`HAL_CAN_RxFifo0FullCallback`、`HAL_CAN_RxFifo1MsgPendingCallback`、`HAL_CAN_RxFifo1FullCallback`。
- 状态和错误：`HAL_CAN_SleepCallback`、`HAL_CAN_WakeUpFromRxMsgCallback`、`HAL_CAN_ErrorCallback`。

本工程 `USE_HAL_CAN_REGISTER_CALLBACKS` 当前为 `0U`，所以**应使用上述弱回调覆盖**。如果另行设为 `1U`，才会编译以下动态注册 API：

```c
HAL_StatusTypeDef HAL_CAN_RegisterCallback(
    CAN_HandleTypeDef *hcan, HAL_CAN_CallbackIDTypeDef CallbackID,
    void (*pCallback)(CAN_HandleTypeDef *hcan));
HAL_StatusTypeDef HAL_CAN_UnRegisterCallback(
    CAN_HandleTypeDef *hcan, HAL_CAN_CallbackIDTypeDef CallbackID);
```

## 七、状态与错误

```c
HAL_CAN_StateTypeDef HAL_CAN_GetState(const CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *hcan);
```

| 返回信息 | 常见值与用法 |
| --- | --- |
| 状态 | `HAL_CAN_STATE_RESET`、`READY`、`LISTENING`、`SLEEP_PENDING`、`SLEEP_ACTIVE`、`ERROR`。 |
| 错误位 | 用 `HAL_CAN_GetError()` 读取后按位检查，例如 `HAL_CAN_ERROR_ACK`、`HAL_CAN_ERROR_BOF`、`HAL_CAN_ERROR_RX_FOV0`。多个错误可以同时存在。 |
| `HAL_CAN_ResetError` | 仅在 `READY` 或 `LISTENING` 状态清除 HAL 句柄中记录的软件错误码并返回 `HAL_OK`；其他状态返回 `HAL_ERROR`。它**不会修复接线、波特率不匹配或物理总线故障**。 |

```c
uint32_t err = HAL_CAN_GetError(&hcan1);
if ((err & HAL_CAN_ERROR_ACK) != 0U) {
    /* 检查对端是否在线、是否有 ACK、位速率和接线 */
}
if ((err & HAL_CAN_ERROR_BOF) != 0U) {
    /* 检查总线关闭原因与恢复策略 */
}
```

## 八、易混淆之处与排查顺序

- 本文使用**新版 HAL CAN**。旧教程中的 `HAL_CAN_Transmit`、`HAL_CAN_Receive` 属于 `stm32f4xx_hal_can_legacy.h`，不要与这里的 `HAL_CAN_AddTxMessage` / `HAL_CAN_GetRxMessage` 混用。
- 本文使用的**新版** bxCAN HAL 不提供 UART 风格的 `HAL_CAN_Transmit_IT`、`HAL_CAN_Receive_IT` 或 CAN DMA 传输 API。发送使用邮箱，接收使用 FIFO；中断只是通知何时处理它们。旧版 legacy 驱动另有前两个函数。
- 收不到帧：依次看 `HAL_CAN_Start` 返回值、过滤器、FIFO 分配、NVIC/IRQ 入口、位速率、收发器与接线。普通模式下只有一个节点也不能完成需要 ACK 的发送。
- 发不出帧：查看 3 个发送邮箱是否长期占用、`HAL_CAN_GetError` 是否出现 ACK/总线错误，并确认 `HAL_CAN_AddTxMessage` 成功仅表示入队。
- 需要循环测试时先选 `CAN_MODE_LOOPBACK`；切回 `CAN_MODE_NORMAL` 后再用两节点和真实总线验证。

## 来源与版本

- 本工程：`base_control.ioc`（芯片、PCLK1、Cube 固件版本）、`Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c`（HAL V1.8.5）、`Core/Inc/stm32f4xx_hal_conf.h`（模块与回调注册开关）、`cmake/stm32cubemx/CMakeLists.txt`（源文件列表）。
- ST 官方同版本源码：[CAN 头文件](https://github.com/STMicroelectronics/stm32f4xx-hal-driver/blob/v1.8.5/Inc/stm32f4xx_hal_can.h)、[CAN 实现](https://github.com/STMicroelectronics/stm32f4xx-hal-driver/blob/v1.8.5/Src/stm32f4xx_hal_can.c)。API 原型、结构体和示例注意事项均按该版本核对。
