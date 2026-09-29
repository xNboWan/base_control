# Crazyflie 实时变量查看机制与迁移思路

本文说明 Crazyflie 如何在程序运行时查看变量，以及如何将这套机制用于 `base_control` 工程的 IMU、遥控和电机回传数据。

分析日期：**2026-09-29**。Crazyflie 部分依据 Bitcraze 官方当前源码和文档；此前提供的 `/media/n/stm/crazyflie-firmware` 路径在本次环境中不可访问，因此尚未核对该本地版本的差异。本文提供实现指导，示例中的遥测模块尚未加入当前工程。

## 1. 先明确要迁移什么

Crazyflie 的实时变量查看主要依靠以下机制：

```text
登记可观察变量 → 上位机选择变量和周期 → 固件周期采样并回传 → 上位机显示、绘图
```

这套机制可以迁移。对于当前工程，建议先通过 UART8 实现变量遥测；如果明确希望使用现有 cfclient，则需要同时兼容它的通信协议和连接流程。

| 路线 | 需要实现的内容 | 适合的目标 |
|---|---|---|
| 简化遥测 | 变量注册、状态快照、UART8 数据帧及配套上位机 | 尽快查看 IMU、遥控和电机数据 |
| 兼容 Crazyflie 生态 | TOC、日志控制、CRTP、兼容传输层及连接初始化 | 复用 cflib，并进一步接入 cfclient |

两条路线都可以保留业务模块的数据读取接口。日志系统额外负责观察和上传数据，任务间仍通过 `imuRead()` 等接口访问状态。

## 2. Crazyflie 的工作过程

```mermaid
flowchart LR
    A["模块内变量"] --> B["变量注册表 TOC"]
    B -- "名称、类型、编号" --> C["电脑端 cfclient"]
    C -- "选择变量、设置周期" --> D["日志控制与周期采样"]
    B --> D
    D -- "CRTP 日志数据包" --> C
```

TOC 是 Table of Contents，可以理解为**可观察变量的目录**。其中包含变量的组名、名称、类型和编号。上位机先获取目录，然后按名称选择变量，建立日志配置并指定回传周期。

配置启动后，固件主动推送数据；上位机可以停止、重新启动或删除配置。[官方日志框架说明](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/logparam/)

电脑端 cfclient 的相关功能如下：

| 功能 | 用途 |
|---|---|
| Log TOC | 查看固件提供了哪些日志变量 |
| Logging Configuration | 选择变量和采样周期 |
| Log Blocks | 管理日志配置的启停及数据保存 |
| Plotter | 将选定配置的数据绘制为曲线 |

操作流程见 [cfclient 用户指南](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/)。Python 程序也可以使用 `LogConfig` 和数据回调，或通过 `SyncLogger` 接收日志。[cflib Python API](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/python_api/)

## 3. 怎样登记变量，又不暴露全局变量

Crazyflie 使用宏登记变量。例如，某个模块内部有一个 yaw 变量：

```c
static float yaw;

LOG_GROUP_START(imu)
LOG_ADD(LOG_FLOAT, yaw, &yaw)
LOG_GROUP_STOP(imu)
```

上位机随后可以按 `imu.yaw` 选择它。这个例子使用 Crazyflie 原生宏，当前工程需要先实现或移植注册模块，才能使用这些宏。

| 登记内容 | 示例 | 作用 |
|---|---|---|
| 组名 | `imu` | 将同一个模块的变量放在一起 |
| 变量名 | `yaw` | 提供人可以识别的名称 |
| 类型 | `LOG_FLOAT` | 确定读取和编码方式 |
| 地址 | `&yaw` | 告诉日志模块从哪里读取值 |

`LOG_GROUP_START()` 将描述数组放入专门的 `.log.*` 链接段。日志初始化时通过链接器提供的 `_log_start`、`_log_stop` 收集这些描述。

**变量仍然可以是模块内部的 `static`，不需要为其他任务添加 `extern`。** 登记地址不会改变变量的 C 语言链接属性；访问这个地址的职责集中在日志模块中。[log.h 源码](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/interface/log.h)

原框架也支持 `LOG_ADD_BY_FUNCTION()`，由回调获取值。对需要通过接口取得数据的模块，可以借鉴这一方式；多字段的一致性还需要整组快照来保证。[函数式日志采集说明](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/logparam/)

## 4. Crazyflie 如何分配日志工作

| 部分 | 主要职责 |
|---|---|
| `logTask()` | 接收 TOC 查询、创建日志块、启动和停止等控制命令 |
| 软件定时器 | 按日志块配置的周期触发采样 |
| `logBlockTimed()` | 将采样工作提交给 worker |
| `logRunBlock()` | 读取变量、添加时间戳、编码数据包 |
| CRTP 发送层 | 将数据包排队并交给通信链路 |

周期采样路径为：

```text
软件定时器 → logBlockTimed() → workerSchedule() → logRunBlock() → crtpSendPacket()
```

日志发送使用非阻塞入队。队列满时允许丢弃日志包，并统计丢包，避免采样工作长期等待通信链路。[log.c 源码](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/log.c)、[crtp.c 源码](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/crtp.c)

这意味着日志用于观察运行状态，实际观察频率受任务调度、数据更新频率和通信带宽影响。

## 5. 当前工程可以复用的基础

当前工程已经有以下基础：

| 已有部分 | 当前作用 | 遥测模块如何利用 |
|---|---|---|
| [imu.c](../User/Modules/Src/imu.c) 中的最新状态队列 | 队列长度为 1，生产者使用 `xQueueOverwrite()` 更新状态 | 保留现有数据发布方式 |
| `imuRead()` | 通过 `xQueuePeek()` 复制最新的 `imuData_t` | 获取整组 IMU 快照 |
| [debug.c](../User/Modules/Src/debug.c) 中的 `debugTask()` | 每 50 ms 读取一次 IMU 数据 | 可作为第一版周期采样的接入位置 |
| [usart.c](../Core/Src/usart.c) 中的 UART8 | 115200 波特率、8N1、收发模式 | 作为遥测物理链路 |

### 5.1 一次读取整个 IMU 快照

建议采样时先调用一次 `imuRead()`，取得整个结构体，再从这份快照中提取加速度、角速度和 yaw：

```c
/* 接入方式示意：打包和发送函数需要另行实现。 */
imuData_t sample;

if (imuRead(&imu, &sample))
{
    /* 从 sample 中提取字段、打包，并提交给发送层。 */
}
```

这样可以使同一个遥测包里的 IMU 字段来自同一份已发布状态。若每个字段分别读取，采样期间可能发生更新，使字段来自不同帧。

还应检查读取结果：队列尚无数据时，不应将未初始化的 `sample` 当作有效数据上传。

后续遥控和电机模块也采用相同思路：各自提供状态读取接口，遥测模块按组取得快照。组与组之间是否同步，应根据业务需求另外确定。

### 5.2 建议的模块分工

| 部分 | 负责什么 |
|---|---|
| IMU、遥控、电机模块 | 维护状态，提供读取接口 |
| 变量注册表 | 登记组名、字段名、类型、单位和获取方式 |
| `telemetryTask` | 按周期获取快照，选取字段并组帧 |
| UART8 发送层 | 管理发送队列和缓冲区，通过中断或 DMA 发送 |
| 上位机 | 解析、显示数值、绘图和保存 |

可新增 `User/Modules/Interface/telemetry.h` 和 `User/Modules/Src/telemetry.c`。沿用当前模块初始化习惯，由 `systemTask` 调用 `telemetryInit()`，后者准备资源并创建遥测任务。以上名称是建议，当前工程尚未提供这些接口。

第一版可以先使用显式注册表，不必立即实现链接段自动注册。等变量数量增加后，再借鉴 Crazyflie 的宏与链接段收集方式；使用该方式时，链接脚本还需保留注册段并提供起止符号。

## 6. 简化 UART8 遥测的实现建议

第一版可让现有 DEBUG 任务承担采样职责，或者创建一个低优先级 `telemetryTask`，每 **50 ms** 获取一次最新状态并发送。之后再按需要增加动态订阅和不同组的采样周期。

建议的数据帧可以包含：

```text
帧头 | 长度 | 消息类型 | 序号 | 时间戳 | 组/日志块编号 | 变量数据 | CRC
```

这是拟定结构，具体字段宽度、字节序和校验算法需要在通信协议中统一规定。

接入时注意以下几点：

- **明确类型和单位**：上下位机一致约定 float、整数及加速度、角速度、角度的单位。
- **区分时间戳含义**：遥测发送或采样时间不等于传感器采集时间。若需要判断样本新鲜度，应由业务模块记录更新时间或样本序号。
- **让发送层持有缓冲区**：提交局部变量或临时组帧数组后，异步发送层应复制数据，或按约定取得缓冲区所有权，直到发送完成。
- **保持发送有界**：遥测队列满时可丢弃包并计数，避免通信拥塞拖住控制或采样任务。
- **统一 UART8 输出**：若文字日志与变量数据共用 UART8，统一帧化并交给同一个发送层调度，避免原始 `printf` 文本混入二进制帧。
- **需要订阅时增加接收处理**：上位机选择变量、修改周期和启停日志都需要双向通信，仅发送数据不能完成这些功能。

中断回调只完成必要的通知和收发处理，解析、组帧和格式化工作放在任务中。任务与中断优先级规则见 [FreeRTOS 任务与中断优先级配置说明](FreeRTOS_任务与中断优先级配置说明.md)。

## 7. 复用 Crazyflie 上位机需要哪些条件

如果希望复用现有 cflib 和 cfclient，固件需要兼容它们期望的日志协议、传输方式及连接初始化流程。只增加 `LOG_ADD()` 注册宏，还不足以建立完整连接。

CRTP 日志协议使用端口 **5**：

| 通道 | 作用 |
|---|---|
| 0 | TOC 查询 |
| 1 | 日志块创建、追加、启动、停止和删除 |
| 2 | 周期日志数据 |

日志数据包含 1 字节日志块编号、3 字节毫秒时间戳，以及最多 **26 字节变量数据**。你的 3 个加速度、3 个角速度和 yaw 若都以 32 位 float 回传，共占 28 字节，因此完整兼容并保持该传输类型时需要拆成多个日志块。自定义 UART 协议则可以按自己的需求设计容量。[官方日志协议](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/crtp/crtp_log/)

日志周期也要按版本确认：旧 `START_BLOCK` 使用 10 ms 单位；当前协议的 `START_BLOCK_V2` 使用毫秒单位。最终可用的粒度和频率还取决于固件、上位机实现、tick 和带宽。[日志控制命令说明](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/crtp/crtp_log/)

### 7.1 官方已经有 UART 接入路线

官方提供 **CPX over UART**：Crazyflie 固件通过 `cpxOverUART2` 接入，Python 端启用串口驱动后使用串口 URI：

```python
cflib.crtp.init_drivers(enable_serial_driver=True)
URI = "serial://ttyUSB0"
```

这个片段用于说明链路选择，前提是设备端已经实现兼容协议。迁移到本项目时，可以将物理串口适配为 UART8；双方波特率和传输格式必须一致。接入 cfclient 时，还需确认其串口驱动启用和连接配置。[官方 UART 接入指南](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/development/uart_communication/)

### 7.2 完整移植的依赖

Crazyflie 原始日志模块依赖软件定时器、worker、CRTP、CRC、配置宏、断言及其他工具。当前 [FreeRTOSConfig.h](../Core/Inc/FreeRTOSConfig.h) 的 `configUSE_TIMERS = 0`，直接移植原始模块需要开启并配置软件定时器服务任务，同时适配其他依赖。

任务优先级、栈大小和队列容量应按本工程重新配置。当前任务优先级范围为 0～7；调用 FreeRTOS 的应用中断应遵守本工程 5～15 的优先级范围，并使用适用的 `…FromISR()` API。

## 8. 推荐实施顺序

如果当前目标是尽快观察数据，按以下顺序推进：

1. **跑通 IMU 回传**：通过 `imuRead()` 获取整组快照，每 50 ms 经 UART8 发送，上位机显示和绘图。
2. **整理注册表**：集中登记字段名称、类型和单位，为遥控、电机状态增加读取入口。
3. **加入订阅控制**：实现上位机查询目录、选择字段、修改周期和启停日志。
4. **验证资源与响应**：观察更新频率、通信丢包、队列占用及控制任务是否受到影响。

如果确定最终需要使用 cfclient，应从开始就采用兼容的 CRTP／CPX 路线：先验证 cflib 的串口连接与日志订阅，再接入 GUI，减少后续替换自定义协议的工作。

持续变化的 IMU、遥控和电机状态适合 Logging；PID 参数等配置值的读取和修改可参考 Crazyflie 的 Parameters 框架。[Logging 与 Parameters 的用途说明](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/logparam/)
