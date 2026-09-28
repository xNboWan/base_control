# STM32 UART 常用 API 参考

> 适用：STM32F4xx HAL 库（`stm32f4xx_hal_uart.h`）
> 本文介绍 USART/UART 外设常用的 HAL 与 LL 库 API，含初始化、阻塞 / 中断 / DMA 三种传输方式、不定长接收（IDLE）、回调函数与常见使用范式。

---

## 目录

1. [初始化与去初始化](#1-初始化与去初始化)
2. [阻塞模式（Polling）](#2-阻塞模式polling)
3. [中断模式（IT）](#3-中断模式it)
4. [DMA 模式](#4-dma-模式)
5. [不定长接收（ReceiveToIdle / IDLE）](#5-不定长接收receivetoidle--idle)
6. [回调函数](#6-回调函数)
7. [状态与错误查询](#7-状态与错误查询)
8. [中止传输（Abort）](#8-中止传输abort)
9. [LL 库 API（简要）](#9-ll-库-api简要)
10. [常见使用范式](#10-常见使用范式)

---

## 1. 初始化与去初始化

### `HAL_UART_Init`

```c
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
```

- 功能：按 `huart` 中的 `Init` 配置初始化 UART 外设。
- 内部会调用 `HAL_UART_MspInit()` 完成 GPIO、时钟、NVIC、DMA 等底层初始化（通常由 CubeMX 生成）。

**配置结构体 `UART_InitTypeDef` 关键字段：**

| 字段 | 说明 |
| --- | --- |
| `BaudRate` | 波特率，如 `115200`、`460800` |
| `WordLength` | 数据位：`UART_WORDLENGTH_8B` / `UART_WORDLENGTH_9B` |
| `StopBits` | 停止位：`UART_STOPBITS_1` / `UART_STOPBITS_2` |
| `Parity` | 校验：`UART_PARITY_NONE` / `UART_PARITY_EVEN` / `UART_PARITY_ODD` |
| `Mode` | 收发方向：`UART_MODE_TX` / `UART_MODE_RX` / `UART_MODE_TX_RX` |
| `HwFlowCtl` | 硬件流控：`UART_HWCONTROL_NONE` / `_RTS` / `_CTS` / `_RTS_CTS` |
| `OverSampling` | 过采样：`UART_OVERSAMPLING_16` / `UART_OVERSAMPLING_8`（8 倍过采样可达更高波特率，但容错更低） |

```c
UART_HandleTypeDef huart1;
huart1.Instance          = USART1;
huart1.Init.BaudRate     = 115200;
huart1.Init.WordLength   = UART_WORDLENGTH_8B;
huart1.Init.StopBits     = UART_STOPBITS_1;
huart1.Init.Parity       = UART_PARITY_NONE;
huart1.Init.Mode         = UART_MODE_TX_RX;
huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
huart1.Init.OverSampling = UART_OVERSAMPLING_16;
if (HAL_UART_Init(&huart1) != HAL_OK) { /* 错误处理 */ }
```

### `HAL_UART_MspInit` / `HAL_UART_DeInit` / `HAL_UART_MspDeInit`

```c
void HAL_UART_MspInit(UART_HandleTypeDef *huart);     // 由 HAL_UART_Init 自动调用
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart);
```

- 在 `MspInit` 中使能时钟、配置 GPIO 复用（TX 复用推挽、RX 浮空输入）、配置 NVIC 与 DMA。
- 在 `MspDeInit` 中关闭时钟、复位 GPIO，保证与 `MspInit` 成对实现。

---

## 2. 阻塞模式（Polling）

阻塞模式会占用 CPU 直到传输完成或超时，适合低频、定长、少量数据。

### `HAL_UART_Transmit` / `HAL_UART_Receive`

```c
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                    const uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive (UART_HandleTypeDef *huart,
                                    uint8_t *pData, uint16_t Size, uint32_t Timeout);
```

- `Size`：要发送 / 接收的字节数。
- `Timeout`：超时时间（单位 ms），超时返回 `HAL_TIMEOUT`。
- **注意**：`HAL_UART_Receive` 会一直阻塞直到收满 `Size` 个字节或超时，因此只适合已知长度的数据。

```c
uint8_t buf[8];
if (HAL_UART_Receive(&huart1, buf, 8, 100) != HAL_OK) {
    // 超时或出错
}
```

---

## 3. 中断模式（IT）

中断模式启动传输后立即返回，完成由中断触发回调，不阻塞 CPU。

### `HAL_UART_Transmit_IT` / `HAL_UART_Receive_IT`

```c
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT (UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
```

- 使用前提：NVIC 已使能 UART 中断，且在 `USARTx_IRQHandler` 中调用 `HAL_UART_IRQHandler(&huartx)`。
- `Receive_IT` 同样收满固定 `Size` 字节才触发完成回调，不能感知“一帧结束”，所以不定长数据应改用第 5 节的 `ReceiveToIdle`。

```c
// stm32f4xx_it.c 中
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}
```

```c
uint8_t rx[8];
HAL_UART_Receive_IT(&huart1, rx, 8);   // 收满 8 字节后进 HAL_UART_RxCpltCallback
```

---

## 4. DMA 模式

DMA 模式适合大批量、连续传输，CPU 开销最低，需要预先在 `MspInit` 中配置 DMA 通道并关联到 `huart->hdmatx` / `huart->hdmarx`。

### `HAL_UART_Transmit_DMA` / `HAL_UART_Receive_DMA`

```c
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_DMA (UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
```

- 若使用 `DMA_CIRCULAR`，`Receive_DMA` 会循环覆盖缓冲区，常用于持续接收流。
- DMA 传输中可暂停 / 恢复 / 停止：

```c
HAL_UART_DMAPause(&huart1);   // 暂停 DMA
HAL_UART_DMAResume(&huart1);  // 恢复 DMA
HAL_UART_DMAStop(&huart1);    // 停止 DMA
```

- 中断回调链：完成进入 `HAL_UART_RxCpltCallback`，半满进入 `HAL_UART_RxHalfCpltCallback`。

---

## 5. 不定长接收（ReceiveToIdle / IDLE）

**这是接收变长帧（如协议帧）的关键 API**。UART 硬件在检测到总线空闲（IDLE）时会产生中断，借此判断“一帧结束”，无需预知长度。

### `HAL_UARTEx_ReceiveToIdle`（阻塞） / `_IT` / `_DMA`

```c
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle(UART_HandleTypeDef *huart, uint8_t *pData,
                                           uint16_t Size, uint16_t *RxLen, uint32_t Timeout);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
```

- `Size`：缓冲区容量（最大可收字节数）。收满 `Size` 或检测到 IDLE 都会结束本次接收。
- `_IT` / `_DMA` 版本结束时在回调 `HAL_UARTEx_RxEventCallback(huart, Size)` 中处理，`Size` 为本次实际收到的字节数。

### `HAL_UARTEx_GetRxEventType`

```c
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *huart);
```

返回本次接收结束的原因，用于区分「收满缓冲区」还是「空闲中断」：

| 宏 | 含义 |
| --- | --- |
| `HAL_UART_RXEVENT_TC` | 收满 `Size` 字节（Transfer Complete） |
| `HAL_UART_RXEVENT_HT` | 收到一半（Half Transfer） |
| `HAL_UART_RXEVENT_IDLE` | 检测到总线空闲（IDLE），一帧结束 |

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE) {
        // 一帧收完，Size 为帧长，处理 g_rx_buf[0..Size-1]
        parse_frame(g_rx_buf, Size);
    }
    // 重新使能接收，等待下一帧
    HAL_UARTEx_ReceiveToIdle_DMA(huart, g_rx_buf, sizeof(g_rx_buf));
}
```

---

## 6. 回调函数

以下回调均为 `__weak` 弱定义，用户在中断服务函数 → HAL 内部处理后重写即可（或通过 `HAL_UART_RegisterCallback` 动态注册）。

### 传输完成回调

| 回调 | 触发时机 |
| --- | --- |
| `HAL_UART_TxCpltCallback` | 发送完成 |
| `HAL_UART_TxHalfCpltCallback` | 发送过半（DMA 模式） |
| `HAL_UART_RxCpltCallback` | 接收完成 |
| `HAL_UART_RxHalfCpltCallback` | 接收过半（DMA 模式） |
| `HAL_UARTEx_RxEventCallback` | `ReceiveToIdle` 接收结束（含 IDLE） |

### 错误 / 中止回调

```c
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);             // 出错（PE/NE/FE/ORE/DMA）
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart);         // 中止完成
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart); // 中止发送完成
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart);  // 中止接收完成
```

---

## 7. 状态与错误查询

```c
HAL_UART_StateTypeDef HAL_UART_GetState(const UART_HandleTypeDef *huart);
uint32_t              HAL_UART_GetError(const UART_HandleTypeDef *huart);
```

**状态**（`HAL_UART_GetState` 返回值）：

| 宏 | 含义 |
| --- | --- |
| `HAL_UART_STATE_RESET` | 未初始化 |
| `HAL_UART_STATE_READY` | 就绪，可发起传输 |
| `HAL_UART_STATE_BUSY` | 内部处理中 |
| `HAL_UART_STATE_BUSY_TX` | 正在发送 |
| `HAL_UART_STATE_BUSY_RX` | 正在接收 |
| `HAL_UART_STATE_BUSY_TX_RX` | 收发同时进行 |
| `HAL_UART_STATE_TIMEOUT` | 超时状态 |
| `HAL_UART_STATE_ERROR` | 出错 |

**错误码**（`HAL_UART_GetError` 返回值，可组合）：

| 宏 | 含义 |
| --- | --- |
| `HAL_UART_ERROR_NONE` | 无错误 |
| `HAL_UART_ERROR_PE` | 校验错误（Parity Error） |
| `HAL_UART_ERROR_NE` | 噪声错误（Noise Error） |
| `HAL_UART_ERROR_FE` | 帧错误（Framing Error，常因波特率不匹配） |
| `HAL_UART_ERROR_ORE` | 溢出错误（Overrun，接收未及时读取） |
| `HAL_UART_ERROR_DMA` | DMA 传输错误 |

```c
if (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY) { /* 忙 */ }
uint32_t err = HAL_UART_GetError(&huart1);
```

---

## 8. 中止传输（Abort）

在中断 / DMA 传输中途需要强制终止时使用（例如超时重试、切换模式）：

```c
HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *huart);          // 中止所有传输
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart);  // 只中止发送
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);   // 只中止接收
// 以及对应的 _IT 非阻塞版本：HAL_UART_Abort_IT / AbortTransmit_IT / AbortReceive_IT
```

- 中止前应确保外设处于 `BUSY` 状态，否则返回 `HAL_ERROR`。
- 中止完成后进入 `HAL_UART_AbortCpltCallback` / `AbortTransmitCpltCallback` / `AbortReceiveCpltCallback`。

---

## 9. LL 库 API（简要）

LL 库（`stm32f4xx_ll_usart.h`）提供寄存器级操作，无状态机与回调，性能更高但需自行管理时序。

```c
LL_USART_Init(USART1, &LL_USART_InitStruct);   // 初始化
LL_USART_Enable(USART1);                       // 使能外设
LL_USART_TransmitData8(USART1, data);          // 写一个字节
LL_USART_ReceiveData8(USART1);                 // 读一个字节
LL_USART_IsActiveFlag_RXNE(USART1);            // 接收寄存器非空
LL_USART_IsActiveFlag_TXE(USART1);             // 发送寄存器空
LL_USART_IsActiveFlag_IDLE(USART1);            // 总线空闲标志
```

---

## 10. 常见使用范式

### 10.1 发送字符串（阻塞，调试最常用）

```c
const char *msg = "hello\r\n";
HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
```

### 10.2 阻塞收定长数据

```c
uint8_t buf[8];
if (HAL_UART_Receive(&huart1, buf, 8, 100) != HAL_OK) {
    // HAL_TIMEOUT / HAL_ERROR
}
```

### 10.3 中断模式收定长数据

```c
uint8_t g_rx[8];
void trigger_rx(void) { HAL_UART_Receive_IT(&huart1, g_rx, 8); }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // 收满 8 字节
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    // 读 HAL_UART_GetError() 定位错误
}
```

### 10.4 DMA + IDLE 收不定长帧（协议通信推荐）

适合收变长协议帧（如 Saber 惯导的 Atom 协议帧）。用 IDLE 判定帧结束，DMA 搬运不占 CPU：

```c
uint8_t g_rx[256];
void uart_rx_start(void)
{
    // 先清错误，再开启 DMA + IDLE 接收
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_rx, sizeof(g_rx));
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        // Size 为本次帧长，解析 g_rx[0..Size-1]
        parse_frame(g_rx, Size);
        // 重新开启接收
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_rx, sizeof(g_rx));
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // ORE 等错误时重新初始化接收，防止停摆
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, g_rx, sizeof(g_rx));
}
```

### 10.5 中断 + IDLE 收不定长（无 DMA 时的替代）

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 处理 g_rx[0..Size-1]
    HAL_UARTEx_ReceiveToIdle_IT(huart, g_rx, sizeof(g_rx));  // 重新开启
}
```

---

## 附：常见错误码速查

| 宏 | 含义 |
| --- | --- |
| `HAL_OK` | 成功 |
| `HAL_BUSY` | 外设忙，上次传输未结束 |
| `HAL_TIMEOUT` | 超时 |
| `HAL_ERROR` | 通用错误 |
| `HAL_UART_ERROR_PE` | 校验错误 |
| `HAL_UART_ERROR_NE` | 噪声错误 |
| `HAL_UART_ERROR_FE` | 帧错误（多为波特率不匹配、接线或时钟配置问题） |
| `HAL_UART_ERROR_ORE` | 接收溢出（数据来了没及时读，建议用 IDLE 或加大缓冲） |
| `HAL_UART_ERROR_DMA` | DMA 传输错误 |

> 提示：出现 `HAL_UART_ERROR_FE` 先核对双方波特率、数据位、停止位、校验位是否一致；出现 `HAL_UART_ERROR_ORE` 说明接收不及时，改用 DMA+IDLE 或及时清 `OREFLAG` 重启接收。
