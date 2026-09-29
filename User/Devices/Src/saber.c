/**
 * @file saber.c
 * @author 李嘉羽 (aa01082241015@gmail.com)
 * @brief saber惯导驱动实现
 * @version 0.1
 * @date 2026-09-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "stdint.h"
#include "string.h"

#include "saber.h"
#include "usart.h"

#include "common.h"

#define PREAMBLE1 0x41
#define PREAMBLE2 0x78
#define TAIL      0x6D

/* cid */
#define CLASS_ID_OPERATION_CMD        0x01
#define CLASS_ID_DEVICE_INFO          0x02
#define CLASS_ID_SENSOR_CONFIG        0x03
#define CLASS_ID_ALGORITHMENGINE      0x04
#define CLASS_ID_COMMUNICATION_CONFIG 0x05
#define CLASS_ID_HOSTCONTROL_CMD      0x06
#define CLASS_ID_FIRMWARE_UPDATE      0x0A
#define CLASS_ID_DEBUG                0x0E

/* mid */

/* 操作类 */
#define CMD_ID_WAKEUP_HOST            0x01
#define CMD_ID_SWITCH_TO_CFG_MODE     0x02
#define CMD_ID_SWITCH_TO_MEASURE_MODE 0x03
#define CMD_ID_SYS_RESET              0x04
#define CMD_ID_CFG_WRITE_TO_FLASH     0x08
#define CMD_ID_CFG_LOAD_FROM_FLASH    0x09
#define CMD_ID_CFG_LOAD_FACTORY       0x0A

/* 通信配置类 */
#define CMD_ID_SET_COMPORT_PARAM       0x01
#define CMD_ID_GET_COMPORT_PARAM       0x02
#define CMD_ID_COMPORT_PARAM_HANDSHAKE 0x10

/* 主机配置类 */
#define CMD_ID_SABER_DATA_PACKET            0x81
#define CMD_ID_SET_CURRENT_ROLLPITCH_OFFSET 0x02
#define CMD_ID_GET_CURRENT_ROLLPITCH_OFFSET 0x03
#define CMD_ID_SET_DRDY_CONFIG              0x06
#define CMD_ID_GET_DRDY_CONFIG              0x07
#define CMD_ID_SET_GPIO_CONFIG              0x08
#define CMD_ID_GET_GPIO_CONFIG              0x09
#define CMD_ID_SET_DATA_PAC_CFG             0x0A
#define CMD_ID_GET_DATA_PAC_CFG             0x0B
#define CMD_ID_SET_SYNCOUT_CFG              0x0C
#define CMD_ID_GET_SYNCOUT_CFG              0x0D

/* 算法配置 */
#define CMD_ID_SET_PACKET_UPDATE_RATE       0x10

/* 数据配置类（仅保留需要的宏） */
/* ---- 原始数据（int16，需乘 gain 换算） ---- */
#define SABER_PID_RAW_ACC  0x8400u /* 原始加速度,    3×int16, g        */
#define SABER_PID_RAW_GYRO 0x8401u /* 原始陀螺,      3×int16, dps      */

/* ---- 校准数据（float，直接是物理量） ---- */
#define SABER_PID_CAL_ACC  0x8800u /* 校准加速度,    3×float, g        */
#define SABER_PID_CAL_GYRO 0x8C00u /* 校准陀螺,      3×float, dps      */

/* ---- 卡尔曼滤波数据（float） ---- */
#define SABER_PID_KALMAN_ACC  0x8801u /* 卡尔曼加速度,  3×float, g        */
#define SABER_PID_KALMAN_GYRO 0x8C01u /* 卡尔曼陀螺,    3×float, dps      */

#define SABER_PID_EULER 0xB001u /* 欧拉角 R/P/Y, 3×float, 度 */


#define ACK_MID(mid)           ((uint8_t)((mid) | 0x80))
#define IMU_COMM_DELAY_TIME_MS 1000


static void saberWakeUpAck(saberCtx_t *saber);
static uint8_t saberAtomBCC(uint8_t *addr, uint16_t len);
static void saberSendFrame(saberCtx_t *saber, uint8_t cid, uint8_t mid, const uint8_t *payload, uint8_t pl);
static uint16_t saberRecvFrame(saberCtx_t *saber, uint8_t *rx_buf, uint16_t buf_size, uint32_t timeout);
static bool saberAckCheck(saberCtx_t *saber, uint8_t cid, uint8_t mid, uint32_t timeout);
static bool saberRecvWakeUpHost(saberCtx_t *saber);
static bool saberSwitchToConfigMode(saberCtx_t *saber, uint32_t timeout);
static bool saberSetUpdateRate(saberCtx_t *saber, uint16_t rate, uint32_t timeout);
static bool saberSetDataPacketConfig(saberCtx_t *saber, uint32_t timeout);
static bool saberSwitchMeasureMode(saberCtx_t *saber, uint32_t timeout);
static bool saberParseDataFrame(saberCtx_t *saber, imuData_t *data);
static bool saberInit(void *ctx);
static bool saberRead(void *ctx, imuData_t *data);


typedef struct
{
    uint16_t pid;
    bool enable;
} saberPktCfg_t;


/* 数据配置注册表 */
static const saberPktCfg_t saber_pkt_table[] = {
    {SABER_PID_KALMAN_ACC,  true},
    {SABER_PID_KALMAN_GYRO, true},
    {SABER_PID_EULER,       true},
};


static uint8_t dma_buf[256] = {0};

imuOps_t saber_ops = {.init = saberInit, .read = saberRead};


bool saberInit(void *ctx)
{
    saberCtx_t *saber = ctx;

    if (!saber || !saber->huart)
        return false;

    uint16_t b = saberRecvWakeUpHost(ctx);
    if (b)
        for (uint8_t i = 0; i < 3; i++)
            saberWakeUpAck(ctx);

    /* 这里的延时不能太短，起码要300ms，否则会导致数据包发送过快，接收不到saber反馈的ack帧
   */
    CHECK(saberSwitchToConfigMode(ctx, IMU_COMM_DELAY_TIME_MS));
    CHECK(saberSetDataPacketConfig(ctx, IMU_COMM_DELAY_TIME_MS));
    CHECK(saberSetUpdateRate(saber, 100u, IMU_COMM_DELAY_TIME_MS)); // 这里频率不可以设置为更大的值，因为更高的发送频率会超出串口的带宽，如果要更高的发送频率，需要同时配置串口和saber的波特率为更高值
    CHECK(saberSwitchMeasureMode(ctx, IMU_COMM_DELAY_TIME_MS));
    HAL_UARTEx_ReceiveToIdle_DMA(saber->huart, dma_buf, sizeof(dma_buf));
    
    return true;
}

bool saberRead(void *ctx, imuData_t *data)
{
    saberCtx_t *saber = ctx;
    return saberParseDataFrame(saber, data);
}


/**
 * @brief bcc校验
 *
 * @param addr 帧指针
 * @param len 帧长度
 * @return uint8_t
 */
uint8_t saberAtomBCC(uint8_t *addr, uint16_t len)
{
    unsigned char *DataPoint;
    DataPoint = addr;
    unsigned char XorData = 0;
    unsigned short DataLength = len;

    while (DataLength--)
    {
        XorData = XorData ^ *DataPoint;
        DataPoint++;
    }

    return XorData;
}

/**
 * @brief 发送一个帧
 *
 * @param cid 操作类
 * @param mid 操作类下的特定命令
 * @param payload 有效载荷指针
 * @param pl payload，载荷长度
 */
void saberSendFrame(saberCtx_t *saber, uint8_t cid, uint8_t mid, const uint8_t *payload, uint8_t pl)
{
    uint8_t frame[262];
    uint16_t i = 0;
    frame[i++] = PREAMBLE1;
    frame[i++] = PREAMBLE2;
    frame[i++] = 0xFF;
    frame[i++] = cid;
    frame[i++] = mid;
    frame[i++] = pl;

    if (payload != NULL)
    {
        for (uint8_t j = 0; j < pl; j++)
            frame[i++] = payload[j];
    }

    uint8_t bcc = saberAtomBCC(frame, i);
    frame[i++] = bcc;
    frame[i++] = TAIL;

    HAL_UART_Transmit(saber->huart, frame, i, 100);
}

/**
 * @brief 接收一个数据帧
 *
 * @param rx_buf 接受缓冲区指针
 * @param buf_size 缓冲区大小
 * @param timeout 超时时间
 * @return uint16_t
 */
uint16_t saberRecvFrame(saberCtx_t *saber, uint8_t *rx_buf, uint16_t buf_size, uint32_t timeout)
{
    uint8_t b = 0;
    uint16_t i = 0;

    /* 找帧头 */
    while (i < 2)
    {
        if (HAL_UART_Receive(saber->huart, &b, 1, timeout) != HAL_OK)
            return 0;

        if (i == 0)
        {
            if (b == PREAMBLE1)
                rx_buf[i++] = b;
        }
        else
        {
            if (b == PREAMBLE2)
                rx_buf[i++] = b;
            else if (b == PREAMBLE1)
                i = 1;
            else
                i = 0;
        }
    }

    /* 找pl */
    if (HAL_UART_Receive(saber->huart, &rx_buf[i], 4, timeout) != HAL_OK)
        return 0;
    i += 4;
    uint8_t pl = rx_buf[5];

    /* pl+8 是整帧的长度 */
    if ((uint16_t)(8 + pl) > buf_size)
        return 0;

    /* 接受完整个数据包 */
    if (HAL_UART_Receive(saber->huart, &rx_buf[i], (uint16_t)(pl + 2), timeout) != HAL_OK)
        return 0;

    i += (uint16_t)(pl + 2);

    /* 帧尾校验 */
    if (rx_buf[i - 1] != TAIL)
        return 0;

    /* bcc校验 */
    if (saberAtomBCC(rx_buf, (uint16_t)(i - 2)) != rx_buf[i - 2])
        return 0;

    /* 返回数据包长度 */
    return i;
}

/**
 * @brief ack校验
 *
 * @param cid
 * @param mid
 * @param timeout
 * @return true
 * @return false
 */
bool saberAckCheck(saberCtx_t *saber, uint8_t cid, uint8_t mid, uint32_t timeout)
{
    uint8_t rx_buf[64];
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout)
    {
        uint16_t n = saberRecvFrame(saber, rx_buf, sizeof(rx_buf), timeout);
        if (n == 0)
            continue;

        if ((rx_buf[3] & 0x0F) == cid && rx_buf[4] == ACK_MID(mid))
        {
            return ((rx_buf[3] >> 4) & 0x0F) == 0;
        }
    }

    return false;
}

/**
 * @brief
 * 在saber上电时，会向主机发送wakeUpHost数据包，该函数用于接收saber发送的数据帧
 *
 * @return true
 * @return false
 */
bool saberRecvWakeUpHost(saberCtx_t *saber)
{
    uint8_t rx_buf[8] = {0};
    uint16_t n = saberRecvFrame(saber, rx_buf, sizeof(rx_buf), 50);
    if (n != 8)
        return false;

    if (rx_buf[3] == CLASS_ID_OPERATION_CMD && rx_buf[4] == CMD_ID_WAKEUP_HOST)
        return true;

    return false;
}

/**
 * @brief 主机向saber发送ack，表示收到数据，发送成功后saber会进入config模式
 *
 */
void saberWakeUpAck(saberCtx_t *saber)
{
    saberSendFrame(saber, CLASS_ID_OPERATION_CMD, ACK_MID(CMD_ID_WAKEUP_HOST), NULL, 0);
}

/**
 * @brief 主机向saber发送消息，让saber进入config模式
 *
 * @return true
 * @return false
 */
bool saberSwitchToConfigMode(saberCtx_t *saber, uint32_t timeout)
{
    uint8_t cid = CLASS_ID_OPERATION_CMD;
    uint8_t mid = CMD_ID_SWITCH_TO_CFG_MODE;

    saberSendFrame(saber, cid, mid, NULL, 0);
    bool ack = saberAckCheck(saber, cid, mid, timeout);
    return ack;
}

/**
 * @brief config模式下配置，获取需要的惯导数据
 *
 * @return true
 * @return false
 */
bool saberSetDataPacketConfig(saberCtx_t *saber, uint32_t timeout)
{
    const uint8_t n = sizeof(saber_pkt_table) / sizeof(saber_pkt_table[0]);
    uint8_t payload[4 * 8];
    uint8_t i = 0;

    if (4 * n > sizeof(payload))
        return false;

    for (uint8_t k = 0; k < n; k++)
    {
        uint16_t pid = saber_pkt_table[k].enable ? saber_pkt_table[k].pid
                                                 : (uint16_t)(saber_pkt_table[k].pid & 0x7FFF);

        payload[i++] = 0xFF;
        payload[i++] = 0xFF;
        payload[i++] = (uint8_t)(pid & 0xFF);
        payload[i++] = (uint8_t)(pid >> 8);
    }

    const uint8_t cid = CLASS_ID_HOSTCONTROL_CMD; /* 0x06 */
    const uint8_t mid = CMD_ID_SET_DATA_PAC_CFG;  /* 0x0A */

    saberSendFrame(saber, cid, mid, payload, (uint8_t)(4 * n));
    bool ack = saberAckCheck(saber, cid, mid, timeout);
    return ack;
}

bool saberSetUpdateRate(saberCtx_t *saber, uint16_t rate, uint32_t timeout)
{
    const uint8_t cid = CLASS_ID_ALGORITHMENGINE;
    const uint8_t mid = CMD_ID_SET_PACKET_UPDATE_RATE;
    const uint16_t raw_update_rate = rate;
    const uint8_t update_rate[2] = {raw_update_rate & 0xFF, raw_update_rate >> 8};
    saberSendFrame(saber, cid, mid, update_rate, 2);
    return saberAckCheck(saber, cid, mid, timeout);
}

/**
 * @brief 发送消息，让saber退出config模式进入maesure模式
 *
 * @return true
 * @return false
 */
bool saberSwitchMeasureMode(saberCtx_t *saber, uint32_t timeout)
{
    uint8_t cid = CLASS_ID_OPERATION_CMD;
    uint8_t mid = CMD_ID_SWITCH_TO_MEASURE_MODE;
    saberSendFrame(saber, cid, mid, NULL, 0);
    bool ack = saberAckCheck(saber, cid, mid, timeout);
    return ack;
}

/**
 * @brief 解析数据帧
 * 
 * @param saber saber上下文指针
 * @param data imu数据缓冲区指针
 * @return true 
 * @return false 
 */
bool saberParseDataFrame(saberCtx_t *saber, imuData_t *data)
{
    uint8_t buf[128];
    uint8_t i = 0;

    float raw_accel[3] = {0};
    float raw_gyro[3] = {0};
    float raw_eular[4] = {0};   //最后一位是保留字段

    uint16_t len = saberRecvFrame(saber, buf, sizeof(buf), 10);
    if (len == 0) 
        return false;
    
    while ((buf[i] != 0x01 || buf[i + 1] != 0x88) && i < len)  i += 1;
    i += 3;

    memcpy(&raw_accel[AXIS_X], &buf[i], sizeof(float)); i += sizeof(float);
    memcpy(&raw_accel[AXIS_Y], &buf[i], sizeof(float)); i += sizeof(float);
    memcpy(&raw_accel[AXIS_Z], &buf[i], sizeof(float)); i += sizeof(float);
    i += 3;

    memcpy(&raw_gyro[AXIS_X], &buf[i], sizeof(float)); i += sizeof(float);
    memcpy(&raw_gyro[AXIS_Y], &buf[i], sizeof(float)); i += sizeof(float);
    memcpy(&raw_gyro[AXIS_Z], &buf[i], sizeof(float)); i += sizeof(float);
    i += 3;

    memcpy(&raw_eular[ROLL], &buf[i], sizeof(float)); i += sizeof(float);
    memcpy(&raw_eular[PITCH], &buf[i], sizeof(float)); i += sizeof(float);
    memcpy(&raw_eular[YAW], &buf[i], sizeof(float)); i += sizeof(float);
    i += 4;
    
    data->accel[AXIS_X] = raw_accel[AXIS_X] * GRAVITY;
    data->accel[AXIS_Y] = raw_accel[AXIS_Y] * GRAVITY;
    data->accel[AXIS_Z] = raw_accel[AXIS_Z] * GRAVITY;

    data->gyro[AXIS_X] = raw_gyro[AXIS_X] * PI / 180.0f;
    data->gyro[AXIS_Y] = raw_gyro[AXIS_Y] * PI / 180.0f;
    data->gyro[AXIS_Z] = raw_gyro[AXIS_Z] * PI / 180.0f;

    data->yaw = raw_eular[YAW] * PI / 180.0f;

    return true;
}

