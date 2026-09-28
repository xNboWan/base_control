# STM32 I2C 常用 API 参考

> 适用：STM32F4xx HAL 库（`stm32f4xx_hal_i2c.h`）
> 本文介绍 I2C 外设常用的 HAL 与 LL 库 API，含初始化、阻塞 / 中断 / DMA 三种传输方式、回调函数与常见使用范式。

---

## 目录

1. [初始化与去初始化](#1-初始化与去初始化)
2. [阻塞模式（Polling）](#2-阻塞模式polling)
3. [中断模式（IT）](#3-中断模式it)
4. [DMA 模式](#4-dma-模式)
5. [回调函数](#5-回调函数)
6. [状态与错误查询](#6-状态与错误查询)
7. [从机专用 API](#7-从机专用-api)
8. [LL 库 API（简要）](#8-ll-库-api简要)
9. [常见使用范式](#9-常见使用范式)

---

## 1. 初始化与去初始化

### `HAL_I2C_Init`

```c
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);
```

- 功能：按 `hi2c` 中的 `I2C_InitTypeDef Init` 配置（时钟速度、寻址模式、占空比等）初始化 I2C 外设。
- 内部会调用 `HAL_I2C_MspInit()` 完成 GPIO、时钟等底层初始化（通常由 CubeMX 生成，用户在其中使能外设时钟、配置引脚复用与 NVIC）。

**配置结构体 `I2C_InitTypeDef` 关键字段：**

| 字段 | 说明 |
| --- | --- |
| `ClockSpeed` | SCL 时钟频率（标准模式 100000 / 快速模式 400000） |
| `DutyCycle` | 快速模式占空比：`I2C_DUTYCYCLE_2` 或 `I2C_DUTYCYCLE_16_9` |
| `OwnAddress1` | 本机从机地址（主机模式可设为 0） |
| `AddressingMode` | `I2C_ADDRESSINGMODE_7BIT` / `I2C_ADDRESSINGMODE_10BIT` |
| `DualAddressMode` | 是否使能双地址（从机） |
| `OwnAddress2` | 第二从机地址 |
| `GeneralCallMode` | 是否响应广播地址 |
| `NoStretchMode` | 是否禁止时钟拉伸 |

```c
I2C_HandleTypeDef hi2c1;
hi2c1.Instance               = I2C1;
hi2c1.Init.ClockSpeed        = 400000;
hi2c1.Init.DutyCycle         = I2C_DUTYCYCLE_2;
hi2c1.Init.OwnAddress1       = 0;
hi2c1.Init.AddressingMode    = I2C_ADDRESSINGMODE_7BIT;
hi2c1.Init.DualAddressMode   = I2C_DUALADDRESS_DISABLE;
hi2c1.Init.OwnAddress2       = 0;
hi2c1.Init.GeneralCallMode   = I2C_GENERALCALL_DISABLE;
hi2c1.Init.NoStretchMode     = I2C_NOSTRETCH_DISABLE;
if (HAL_I2C_Init(&hi2c1) != HAL_OK) { /* 错误处理 */ }
```

### `HAL_I2C_MspInit`

```c
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);
```

- 功能：底层初始化回调，由 `HAL_I2C_Init` 自动调用。用户在此使能 `__HAL_RCC_I2Cx_CLK_ENABLE()`、配置 GPIO 复用（开漏输出、上拉）、配置 DMA/NVIC。
- 注意：此函数**不会被** `HAL_I2C_DeInit` 之外自动调用，需保证 MspInit/MspDeInit 成对实现。

### `HAL_I2C_DeInit` / `HAL_I2C_MspDeInit`

```c
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c);
```

- 功能：去初始化外设，复位到默认状态；`MspDeInit` 中应关闭时钟、复位 GPIO（由用户实现）。

---

## 2. 阻塞模式（Polling）

阻塞模式会占用 CPU 直到传输完成或超时，适合简单、低频的场景。

### `HAL_I2C_Master_Transmit` / `HAL_I2C_Master_Receive`

```c
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c,
                                          uint16_t DevAddress, uint8_t *pData,
                                          uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive (I2C_HandleTypeDef *hi2c,
                                          uint16_t DevAddress, uint8_t *pData,
                                          uint16_t Size, uint32_t Timeout);
```

- 功能：主机**不带寄存器地址**的发送 / 接收。
- `DevAddress`：从机 7 位地址需**左移 1 位**（例：从机 0x68 → 传 `0x68 << 1`）；10 位地址使用宏 `I2C_ADDRESSINGMODE_10BIT` 时同样处理。
- `Timeout`：超时时间（单位 ms），超时返回 `HAL_TIMEOUT`。

### `HAL_I2C_Mem_Write` / `HAL_I2C_Mem_Read`

```c
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                    uint16_t MemAddress, uint16_t MemAddSize,
                                    uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read (I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                    uint16_t MemAddress, uint16_t MemAddSize,
                                    uint8_t *pData, uint16_t Size, uint32_t Timeout);
```

- 功能：**带寄存器（内存）地址**的读写，是操作传感器寄存器最常用的 API。
- `MemAddress`：器件内部寄存器地址。
- `MemAddSize`：寄存器地址宽度，`I2C_MEMADD_SIZE_8BIT` 或 `I2C_MEMADD_SIZE_16BIT`。

```c
// 从 0x68 的寄存器 0x3B 读 6 字节
uint8_t buf[6];
if (HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x3B, I2C_MEMADD_SIZE_8BIT,
                     buf, 6, 100) != HAL_OK) {
    // 处理错误
}
```

### `HAL_I2C_IsDeviceReady`

```c
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c,
                                        uint16_t DevAddress, uint32_t Trials,
                                        uint32_t Timeout);
```

- 功能：探测从机是否就绪（发送地址后检测 ACK），常用于上电后确认器件存在。

---

## 3. 中断模式（IT）

中断模式启动传输后立即返回，传输完成由中断触发回调，不阻塞 CPU。

| API | 功能 |
| --- | --- |
| `HAL_I2C_Master_Transmit_IT` | 主机发送（无寄存器地址） |
| `HAL_I2C_Master_Receive_IT` | 主机接收（无寄存器地址） |
| `HAL_I2C_Mem_Write_IT` | 主机写寄存器 |
| `HAL_I2C_Mem_Read_IT` | 主机读寄存器 |

```c
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c,
                                             uint16_t DevAddress, uint8_t *pData,
                                             uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                      uint16_t MemAddress, uint16_t MemAddSize,
                                      uint8_t *pData, uint16_t Size);
```

- 使用前提：NVIC 中已使能 I2C 事件 / 错误中断（`HAL_NVIC_EnableIRQ(I2Cx_EV_IRQn)`、`..._ER_IRQn`）。
- 传输完成后在回调（见第 5 节）中处理数据。
- 中断模式下每次调用前应确认外设处于 `HAL_I2C_STATE_READY`，否则会返回 `HAL_BUSY`。

---

## 4. DMA 模式

DMA 模式适合大批量数据传输，CPU 开销最低，需要预先配置 DMA 通道（`hdmatx` / `hdmarx`）。

| API | 功能 |
| --- | --- |
| `HAL_I2C_Master_Transmit_DMA` | 主机 DMA 发送 |
| `HAL_I2C_Master_Receive_DMA` | 主机 DMA 接收 |
| `HAL_I2C_Mem_Write_DMA` | 主机 DMA 写寄存器 |
| `HAL_I2C_Mem_Read_DMA` | 主机 DMA 读寄存器 |

```c
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                       uint16_t MemAddress, uint16_t MemAddSize,
                                       uint8_t *pData, uint16_t Size);
```

- 使用前提：`MspInit` 中完成 DMA 流配置（通道、方向、优先级、`DMA_NORMAL` / `DMA_CIRCULAR`）并关联到 `hi2c->hdmatx` / `hi2c->hdmarx`。
- 若使用 `DMA_CIRCULAR`，需在回调 `HAL_I2C_MemRxCpltCallback` 中再次调用读取 API 以持续接收。
- DMA 中断回调链：`HAL_I2C_MemRxCpltCallback`（半满为 `HAL_I2C_MemRxHalfCpltCallback`）。

---

## 5. 回调函数

以下回调均为 `__weak` 弱定义，用户在 `stm32f4xx_it.c` 的中断服务函数（`I2Cx_EV_IRQHandler` / `I2Cx_ER_IRQHandler` / DMA IRQ）→ HAL 内部处理后，由用户重写以完成业务逻辑。

### 传输完成回调

| 回调 | 触发时机 |
| --- | --- |
| `HAL_I2C_MasterTxCpltCallback` | 主机（无地址）发送完成 |
| `HAL_I2C_MasterRxCpltCallback` | 主机（无地址）接收完成 |
| `HAL_I2C_MemTxCpltCallback` | 主机写寄存器完成 |
| `HAL_I2C_MemRxCpltCallback` | 主机读寄存器完成 |
| `HAL_I2C_SlaveTxCpltCallback` | 从机发送完成 |
| `HAL_I2C_SlaveRxCpltCallback` | 从机接收完成 |

```c
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // 数据已在全局缓冲区中，置标志或直接处理
    g_rx_done = 1;
}
```

### 错误 / 中止回调

```c
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);   // 传输出错（BUS_ERROR、AF、ARLO、OVR 等）
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c); // 中止完成
```

### 从机事件回调

```c
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
                          uint16_t AddrMatchCode);      // 主机寻址到本机
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c); // 监听（Listen）完成
void HAL_I2C_MemListenCpltCallback(I2C_HandleTypeDef *hi2c); // 监听内存接收完成
```

---

## 6. 状态与错误查询

```c
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c);
uint32_t            HAL_I2C_GetError(I2C_HandleTypeDef *hi2c);
```

- `HAL_I2C_GetState` 返回当前状态：`HAL_I2C_STATE_READY` / `HAL_I2C_STATE_BUSY` / `HAL_I2C_STATE_BUSY_TX` / `HAL_I2C_STATE_BUSY_RX` / `HAL_I2C_STATE_LISTEN` 等。
- `HAL_I2C_GetError` 返回错误码（可组合）：`HAL_I2C_ERROR_NONE`、`HAL_I2C_ERROR_BERR`（总线错误）、`HAL_I2C_ERROR_AF`（应答失败）、`HAL_I2C_ERROR_ARLO`（仲裁丢失）、`HAL_I2C_ERROR_OVR`（过载 / 溢出）、`HAL_I2C_ERROR_TIMEOUT`、`HAL_I2C_ERROR_DMA`。

```c
if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) { /* 忙 */ }
```

---

## 7. 从机专用 API

| API | 功能 |
| --- | --- |
| `HAL_I2C_Slave_Transmit` / `_IT` / `_DMA` | 从机发送 |
| `HAL_I2C_Slave_Receive` / `_IT` / `_DMA` | 从机接收 |
| `HAL_I2C_EnableListen_IT` | 开启监听模式（等待主机寻址，不主动拉低时钟） |
| `HAL_I2C_DisableListen_IT` | 关闭监听模式 |

```c
HAL_StatusTypeDef HAL_I2C_Slave_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData,
                                           uint16_t Size);
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *hi2c);
```

> F4 系列还提供从机**连续传输**（Sequential）API，如 `HAL_I2C_Slave_Seq_Transmit_IT` / `HAL_I2C_Slave_Seq_Receive_IT`，用于从机连续收发不定长数据，配合 `HAL_I2C_AddrCallback` 判断读写方向。

---

## 8. LL 库 API（简要）

LL 库（`stm32f4xx_ll_i2c.h`）提供更底层、更轻量的寄存器级操作，无状态机与回调，性能更高但需自行管理时序。

```c
LL_I2C_Init(I2C1, &LL_I2C_InitStruct);      // 初始化（结构体字段与 HAL 类似）
LL_I2C_SetMode(I2C1, LL_I2C_MODE_I2C);      // 设置 I2C 模式
LL_I2C_Enable(I2C1);                        // 使能外设

LL_I2C_GenerateStartCondition(I2C1);        // 产生起始条件
LL_I2C_TransmitData8(I2C1, data);           // 写数据
LL_I2C_ReceiveData8(I2C1);                  // 读数据
LL_I2C_GenerateStopCondition(I2C1);         // 产生停止条件

while (!LL_I2C_IsActiveFlag_SB(I2C1));      // 等待起始位
LL_I2C_ClearFlag_AF(I2C1);                  // 清除标志
```

常用标志 / 状态查询：

```c
LL_I2C_IsActiveFlag_SB(I2C1);   // Start Bit
LL_I2C_IsActiveFlag_ADDR(I2C1); // 地址已发送
LL_I2C_IsActiveFlag_TXE(I2C1);  // 发送数据寄存器空
LL_I2C_IsActiveFlag_RXNE(I2C1); // 接收数据寄存器非空
LL_I2C_IsActiveFlag_AF(I2C1);   // 应答失败
LL_I2C_IsActiveFlag_BERR(I2C1); // 总线错误
```

---

## 9. 常见使用范式

### 9.1 读取传感器寄存器（阻塞，最常用）

```c
// 读 IMU 加速度寄存器（例：0x68 器件，寄存器 0x3B，共 6 字节）
uint8_t raw[6];
HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x3B,
                                        I2C_MEMADD_SIZE_8BIT, raw, 6, 100);
if (st != HAL_OK) {
    // HAL_TIMEOUT / HAL_BUSY / HAL_ERROR
}
```

### 9.2 写入传感器配置寄存器

```c
uint8_t cfg = 0x18; // 示例配置值
HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x1B, I2C_MEMADD_SIZE_8BIT, &cfg, 1, 100);
```

### 9.3 中断模式非阻塞读取

```c
uint8_t g_rx[6];
volatile uint8_t g_rx_ready = 0;

void trigger_read(void)
{
    HAL_I2C_Mem_Read_IT(&hi2c1, 0x68 << 1, 0x3B, I2C_MEMADD_SIZE_8BIT, g_rx, 6);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    g_rx_ready = 1;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    // 可选：读 HAL_I2C_GetError() 定位错误，必要时重试
}
```

### 9.4 器件探测

```c
if (HAL_I2C_IsDeviceReady(&hi2c1, 0x68 << 1, 3, 100) != HAL_OK) {
    // 器件未就绪或地址错误
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
| `HAL_I2C_ERROR_BERR` | 总线错误（起始/停止条件时序异常） |
| `HAL_I2C_ERROR_AF` | 无应答（地址错或器件掉线） |
| `HAL_I2C_ERROR_ARLO` | 仲裁丢失 |
| `HAL_I2C_ERROR_OVR` | 过载 / 接收溢出 |
| `HAL_I2C_ERROR_TIMEOUT` | 时钟拉伸 / 传输超时 |

> 提示：若出现 `HAL_I2C_ERROR_AF`，先确认从机地址是否左移 1 位、上拉电阻是否正常、器件是否上电就绪。
