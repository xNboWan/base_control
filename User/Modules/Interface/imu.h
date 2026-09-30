/**
 * @file imu.h
 * @李嘉羽 (aa01082241015@gmail.com)
 * @brief imu抽象层头文件，该文件为上层对imu的调用提供了统一的接口
 * @version 0.1
 * @date 2026-09-29
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

#include "common.h"

typedef struct
{
    float accel[3];    
    float gyro[3]; 
    float yaw;   
} imuData_t;

typedef struct
{
    bool (*init)(void *ctx);
    bool (*read)(void *ctx, imuData_t *pdata); 
} imuOps_t;

typedef struct
{
    const imuOps_t *ops;
    void *ctx;
} imu_t;

/**
 * @brief imu初始化，该函数会自动创建一个深度为1的队列，和imu任务，编写该函数对应的操作函数时，只需要实现硬件的初始 *                  化逻辑，无需创建任务
 * 
 * @param imu 惯导实例
 * @return true 
 * @return false 
 */
bool imuInit(imu_t *imu);

/**
 * @brief 上层获取imu数据的接口
 * 
 * @param imu 惯导实例
 * @param data 数据存放的指针
 * @return true 
 * @return false 
 */
bool imuRead(imu_t *imu, imuData_t *pdata);

extern imu_t imu;

#endif