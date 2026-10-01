#include "stdbool.h"

#include "m3508.h"
#include "bsp_can.h"
#include "static_mem.h"
#include "semphr.h"
#include <string.h>

#define SEND_MSG_LEN 8
#define IDENTIFIER1  0x200
#define IDENTIFIER2  0x1FF

static bool m3508Init(void *ctx);
static bool m3508Read(void *ctx, motorData_t *pdata);
static bool m3508Write(void *ctx, motorCmd_t *cmd);
bool m3508Send(void *ctx);

typedef struct
{
    motorCmd_t cmd;
    uint8_t id;
} m3508CmdMsg_t;

motorOps_t m3508_ops = {.init = m3508Init,
    .read = m3508Read,
    .write = m3508Write,
    .send = m3508Send};

static motorCmd_t m3508_cmd[8] = {0};
static StaticSemaphore_t cmd_mutex_buffer;
static SemaphoreHandle_t cmd_mutex = NULL;
static volatile uint8_t motor_feedback_buf[32] = {0};
static volatile bool motor_feedback_valid[4] = {false};

bool is_can_start = false;

bool m3508Init(void *ctx)
{
    m3508Ctx_t *motor_ctx = ctx;

    if (motor_ctx == NULL || motor_ctx->hcan == NULL)
        return false;

    if (motor_ctx->id < 1 || motor_ctx->id > 8)
        return false;

    if (!is_can_start)
    {
        if (!canStart(motor_ctx->hcan))
            return false;
        if (cmd_mutex == NULL)
        {
            cmd_mutex = xSemaphoreCreateMutexStatic(&cmd_mutex_buffer);

            if (cmd_mutex == NULL)
                return false;
        }
        is_can_start = true;
    }
    return true;
}

static bool m3508Read(void *ctx, motorData_t *pdata)
{
    m3508Ctx_t *motor_ctx = (m3508Ctx_t *)ctx;

    if (motor_ctx == NULL || motor_ctx->hcan == NULL || motor_ctx->hcan->Instance != CAN1
        || pdata == NULL || motor_ctx->id < 1 || motor_ctx->id > 8)
    {
        return false;
    }

    uint8_t index = (uint8_t)(motor_ctx->id - 1);
    uint8_t offset = (uint8_t)(index * 8);
    uint8_t raw[8];
    bool valid;

    taskENTER_CRITICAL();

    valid = motor_feedback_valid[index];
    if (valid)
    {
        for (uint8_t i = 0; i < 8U; i++)
            raw[i] = motor_feedback_buf[offset + i];
    }

    taskEXIT_CRITICAL();

    if (!valid)
        return false;

    uint16_t angle_count = (uint16_t)(((uint16_t)raw[0] << 8) | raw[1]);

    int16_t speed_rpm = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);

    int16_t current_count = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);

    pdata->omega = (float)angle_count / 4096.0f;
    pdata->d_omega = (float)speed_rpm / 30.0f;

    /* 按反馈电流采用 ±16384 ↔ ±20 A 的比例估算 */
    float current_a = (float)current_count * (20.0f / 16384.0f);
    pdata->lq = current_a;

    pdata->temperature = (int16_t)raw[6];

    return true;
}

bool m3508Write(void *ctx, motorCmd_t *cmd)
{
    m3508Ctx_t *m3508_ctx = ctx;

    if (m3508_ctx->id < 1 || m3508_ctx->id > 8)
        return false;
    if (cmd->torque < -16384 || cmd->torque > 16384)
        return false;

    if (xSemaphoreTake(cmd_mutex, pdMS_TO_TICKS(1)) != pdTRUE)
        return false;
    m3508_cmd[m3508_ctx->id - 1] = *cmd;
    xSemaphoreGive(cmd_mutex);

    return true;
}

bool m3508Send(void *ctx)
{
    m3508Ctx_t *motor_ctx = ctx;

    if (motor_ctx == NULL || motor_ctx->hcan == NULL || cmd_mutex == NULL)
        return false;

    if (motor_ctx->id < 1 || motor_ctx->id > 8)
        return false;

    uint8_t group = (motor_ctx->id - 1) / 4;

    motorCmd_t snapshot[4];

    if (xSemaphoreTake(cmd_mutex, pdMS_TO_TICKS(1)) != pdTRUE)
        return false;

    memcpy(snapshot, &m3508_cmd[group * 4], sizeof(snapshot));

    xSemaphoreGive(cmd_mutex);

    uint8_t tx_data[8];

    for (uint8_t i = 0; i < 4; i++)
    {
        uint16_t raw = (uint16_t)snapshot[i].torque;

        tx_data[2 * i] = (uint8_t)(raw >> 8);
        tx_data[2 * i + 1] = (uint8_t)(raw & 0xFF);
    }

    uint32_t identifier = group == 0 ? IDENTIFIER1 : IDENTIFIER2;

    return canSend(motor_ctx->hcan, identifier, tx_data, sizeof(tx_data));
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL || hcan->Instance != CAN1)
        return;

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        CAN_RxHeaderTypeDef rx;
        uint8_t rx_buf[8];

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, rx_buf) != HAL_OK)
            break;

        if (rx.IDE != CAN_ID_STD || rx.RTR != CAN_RTR_DATA || rx.DLC != 8U || rx.StdId < 0x201U
            || rx.StdId > 0x204U)
        {
            continue;
        }

        uint8_t index = (uint8_t)(rx.StdId - 0x201U);
        uint8_t offset = (uint8_t)(index * 8U);

        for (uint8_t i = 0; i < 8U; i++)
            motor_feedback_buf[offset + i] = rx_buf[i];

        /* 数据复制完成后，再标记为有效 */
        motor_feedback_valid[index] = true;
    }
}