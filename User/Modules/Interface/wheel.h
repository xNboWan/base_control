/**
 * @file wheel.h
 * @author 李嘉羽 (aa01082241015@gmail.com)
 * @brief 轮组模块接口文件
 * @version 0.1
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef WHEEL_H
#define WHEEL_H

#include "stdint.h"
#include "stdbool.h"

#include "pid.h"
#include "motor.h"

typedef enum
{
    FORWARD = 1,
    REVERSE = -1
} wheelDir_e;

typedef struct
{
    int8_t dir;
    float wheel_radius;
    float v_max;
    float gear_ratio;

    pidCfg_t pid_cfg;
    motor_t motor;
} wheelCfg_t;

typedef struct
{
    float d_theta;
} wheelCmd_t;

typedef struct
{
    float theta;  // 绝对位置，当前未实现角度累计
    float d_theta;
    float target_d_theta;
    float v;
    uint16_t temperature;
} wheelData_t;

typedef struct
{
    wheelData_t data;
    wheelCfg_t cfg;
    pid_t pid;
} wheel_t;

typedef struct
{
    uint8_t wheel_num;
    wheel_t *wheel;
} wheelGroup_t;

/**
 * @brief 轮子初始化
 * 
 * @param wheel 轮子句柄
 * @param cfg 轮子配置
 * @return true 
 * @return false 
 */
bool wheelInit(wheel_t *wheel, wheelCfg_t *cfg);

/**
 * @brief 获取轮子运行参数
 * 
 * @param wheel 轮子句柄
 * @param pdata 数据缓冲区
 * @return true 
 * @return false 
 */
bool wheelRead(wheel_t *wheel, wheelData_t *pdata);

/**
 * @brief 给轮子下发指令，该函数只负责将指令写入发送缓冲区，不会将指令下发电机
 * 
 * @param wheel 轮子句柄
 * @param cmd 轮子指令缓冲区
 * @param dt 用于pid计算，和轮子控制周期一致
 * @return true 
 * @return false 
 */
bool wheelWrite(wheel_t *wheel, wheelCmd_t *cmd, float dt);

/**
 * @brief 将指令下发给电机
 * 
 * @param wheel 轮子句柄
 * @return true 
 * @return false 
 */
bool wheelSend(wheel_t *wheel);

/**
 * @brief 轮组指令写入和发送
 * 
 * @param wheel_group 轮组句柄
 * @param cmd 指令列表，该指令必须是一个连续的数组，且长度和轮子数量一致
 * @param dt 用于pid计算，和轮子控制周期一致
 * @return true 
 * @return false 
 */
bool wheelGroupWriteAndSend(wheelGroup_t *wheel_group, wheelCmd_t *cmd, float dt);

extern wheel_t wheel[4];

#endif