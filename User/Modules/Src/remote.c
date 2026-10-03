#include "remote.h"
#include "chassis.h"
#include "crc.h"
#include "usart.h"

#include "static_mem.h"
#include "stream_buffer.h"

#include "math.h"
#include <stdint.h>
#include <string.h>

#define HEAD1 0x72u
#define HEAD2 0x65u
#define HEAD2_MODE 0x6du
#define TAIL  0x64u

#define REMOTE_FRAME_LEN   23u
#define REMOTE_MODE_PAYLOAD_LEN 20u
#define REMOTE_MODE_FRAME_LEN   27u
#define REMOTE_RX_STORAGE_LEN 256u
#define REMOTE_RECV_TIMEOUT_MS 20u

/* 原 23 字节：0x72 | 0x65 | vx | vy | yaw | d_yaw | CRC32 | 0x64，角速度模式。
 * 新 27 字节：0x72 | 0x6d | vx | vy | yaw | d_yaw | mode(uint32_t) | CRC32 | 0x64。
 * mode: HEADLOCK=0，HEADFREE=1；CRC 覆盖全部 16/20 字节载荷，包括 mode。
 * float 为 IEEE 754 单精度；float、mode 和 CRC32 均为小端。
 * 每四字节按小端组成 uint32_t 后送入 F427 CRC。
 * 多项式 0x04C11DB7，初值 0xFFFFFFFF，不反射、不做最终异或。
 */
_Static_assert(sizeof(float) == 4u, "Remote protocol requires 32-bit float");

static uint8_t remote_buf[REMOTE_MODE_FRAME_LEN];
static uint8_t remote_pending[REMOTE_MODE_FRAME_LEN];
static uint16_t remote_pending_count;
static uint8_t remote_rx_byte;
static uint8_t remote_rx_storage[REMOTE_RX_STORAGE_LEN];
static StaticStreamBuffer_t remote_rx_control;
static StreamBufferHandle_t remote_rx_stream;
static TaskHandle_t remote_task;
static bool is_init = false;

static void remoteTask(void *arg);
static uint16_t remoteRecvFrame(uint8_t *rx_buf, uint16_t buf_size, uint32_t timeout);
static bool remoteParseFrame(const uint8_t *frame, uint16_t frame_size);
static bool remoteCheckFrame(const uint8_t *frame, uint16_t frame_size);

bool remoteInit(void)
{
    if (is_init)
        return true;

    remote_rx_stream = xStreamBufferCreateStatic(sizeof(remote_rx_storage), 1u,
                                                remote_rx_storage, &remote_rx_control);
    if (!remote_rx_stream)
        return false;

    /* UART8 回调使用 FromISR API，优先级必须满足 FreeRTOS 的限制。
     * 覆盖当前 CubeMX 生成的优先级 0。
     */
    HAL_NVIC_SetPriority(UART8_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(UART8_IRQn);

    if (HAL_UART_Receive_IT(&huart8, &remote_rx_byte, 1u) != HAL_OK)
    {
        vStreamBufferDelete(remote_rx_stream);
        remote_rx_stream = NULL;
        return false;
    }

    STATIC_MEM_TASK_ALLOC(remoteTask, 256);
    remote_task = STATIC_MEM_TASK_CREATE(remoteTask, remoteTask, "REMOTE", NULL, 1);
    if (!remote_task)
    {
        (void)HAL_UART_AbortReceive(&huart8);
        vStreamBufferDelete(remote_rx_stream);
        remote_rx_stream = NULL;
        return false;
    }

    is_init = true;
    return true;
}

static void remoteTask(void *arg)
{
    (void)arg;

    for (;;)
    {
        uint16_t frame_size = remoteRecvFrame(remote_buf, sizeof(remote_buf), REMOTE_RECV_TIMEOUT_MS);
        if (frame_size != 0u)
            (void)remoteParseFrame(remote_buf, frame_size);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart8 || !remote_rx_stream)
        return;

    BaseType_t woken = pdFALSE;
    uint8_t byte = remote_rx_byte;
    (void)HAL_UART_Receive_IT(huart, &remote_rx_byte, 1u);
    (void)xStreamBufferSendFromISR(remote_rx_stream, &byte, 1u, &woken);
    portYIELD_FROM_ISR(woken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart8 || !remote_rx_stream)
        return;

    /* UART8 使用中断接收，没有 RX DMA；清除错误后重新接收。 */
    (void)HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    (void)HAL_UART_Receive_IT(huart, &remote_rx_byte, 1u);
}

static uint32_t remoteReadU32LE(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static uint16_t remoteFrameLength(uint8_t head2)
{
    if (head2 == HEAD2) return REMOTE_FRAME_LEN;
    if (head2 == HEAD2_MODE) return REMOTE_MODE_FRAME_LEN;
    return 0u;
}

static bool remoteCheckFrame(const uint8_t *frame, uint16_t frame_size)
{
    if (!frame || frame_size < REMOTE_FRAME_LEN ||
        frame_size != remoteFrameLength(frame[1]) ||
        frame[0] != HEAD1 || frame[frame_size - 1u] != TAIL)
        return false;

    uint16_t payload_len = frame_size - 7u;
    uint32_t words[REMOTE_MODE_PAYLOAD_LEN / 4u];
    for (uint8_t i = 0; i < payload_len / 4u; i++)
        words[i] = remoteReadU32LE(&frame[2u + 4u * i]);

    /* 长度单位为 uint32_t 的个数；Calculate 会复位后计算这一帧。 */
    uint32_t crc = HAL_CRC_Calculate(&hcrc, words, payload_len / 4u);
    return crc == remoteReadU32LE(&frame[2u + payload_len]);
}

/* 坏帧可能吞入下一帧的帧头；保留最早的候选头或末尾的单字节头。 */
static void remoteDiscardCandidate(void)
{
    uint16_t next = 1u;
    while (next + 1u < remote_pending_count &&
           (remote_pending[next] != HEAD1 || remoteFrameLength(remote_pending[next + 1u]) == 0u))
        next++;

    if (next + 1u < remote_pending_count)
    {
        remote_pending_count -= next;
        memmove(remote_pending, &remote_pending[next], remote_pending_count);
    }
    else if (remote_pending_count != 0u && remote_pending[remote_pending_count - 1u] == HEAD1)
    {
        remote_pending[0] = HEAD1;
        remote_pending_count = 1u;
    }
    else
        remote_pending_count = 0u;
}

/* 在 timeout 毫秒内寻找一帧有效数据，返回实际 23/27 字节长度，失败为 0。
 * 等待串口时让出 CPU；不完整帧在超时后丢弃。
 * CRC 错误后的重新同步保留额外字节，可连续接收两种长度的帧。
 */
static uint16_t remoteRecvFrame(uint8_t *rx_buf, uint16_t buf_size, uint32_t timeout)
{
    if (!rx_buf || buf_size < REMOTE_MODE_FRAME_LEN || !remote_rx_stream)
        return 0u;

    TickType_t start = xTaskGetTickCount();
    TickType_t budget = timeout == HAL_MAX_DELAY ? portMAX_DELAY : pdMS_TO_TICKS(timeout);
    if (timeout != 0u && budget == 0u)
        budget = 1u;

    for (;;)
    {
        while (remote_pending_count >= 2u)
        {
            uint16_t frame_size = remoteFrameLength(remote_pending[1]);
            if (remote_pending[0] != HEAD1 || frame_size == 0u)
            {
                remoteDiscardCandidate();
                continue;
            }
            if (remote_pending_count < frame_size)
                break;
            if (remoteCheckFrame(remote_pending, frame_size))
            {
                memcpy(rx_buf, remote_pending, frame_size);
                remote_pending_count -= frame_size;
                memmove(remote_pending, &remote_pending[frame_size], remote_pending_count);
                return frame_size;
            }
            remoteDiscardCandidate();
        }

        TickType_t remaining = budget;
        if (budget != 0u && budget != portMAX_DELAY)
        {
            TickType_t elapsed = xTaskGetTickCount() - start;
            if (elapsed >= budget)
            {
                remote_pending_count = 0u;
                return 0u;
            }
            remaining = budget - elapsed;
        }

        uint8_t byte;
        if (xStreamBufferReceive(remote_rx_stream, &byte, 1u, remaining) != 1u)
        {
            remote_pending_count = 0u;
            return 0u;
        }

        if (remote_pending_count == 0u)
        {
            if (byte == HEAD1)
                remote_pending[remote_pending_count++] = byte;
            continue;
        }
        if (remote_pending_count == 1u)
        {
            if (remoteFrameLength(byte) != 0u)
                remote_pending[remote_pending_count++] = byte;
            else if (byte != HEAD1)
                remote_pending_count = 0u;
            continue;
        }

        remote_pending[remote_pending_count++] = byte;
    }
}

static bool remoteParseFrame(const uint8_t *frame, uint16_t frame_size)
{
    if (!remoteCheckFrame(frame, frame_size))
        return false;

    /* 按字段读取，避免直接转换未对齐的载荷地址或依赖结构体布局。 */
    uint32_t bits[4];
    for (uint8_t i = 0; i < 4u; i++)
        bits[i] = remoteReadU32LE(&frame[2u + 4u * i]);

    uint32_t mode = frame_size == REMOTE_FRAME_LEN ? HEADFREE : remoteReadU32LE(&frame[18u]);
    if (mode != HEADLOCK && mode != HEADFREE)
        return false;
    chassisCmd_t cmd = {.mode = (chassisMode)mode};
    memcpy(&cmd.vx, &bits[0], sizeof(cmd.vx));
    memcpy(&cmd.vy, &bits[1], sizeof(cmd.vy));
    memcpy(&cmd.yaw, &bits[2], sizeof(cmd.yaw));
    memcpy(&cmd.d_yaw, &bits[3], sizeof(cmd.d_yaw));

    if (!isfinite(cmd.vx) || !isfinite(cmd.vy) ||
        !isfinite(cmd.yaw) || !isfinite(cmd.d_yaw))
        return false;

    return chassisSetCommand(&cmd);
}
