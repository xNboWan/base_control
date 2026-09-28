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
    bool (*read)(void *ctx, imuData_t *data); 
} imuOps_t;

typedef struct
{
    const imuOps_t *ops;
    void *ctx;
} imu_t;

bool imuInit(imu_t *imu);
bool imuRead(imu_t *imu, imuData_t *data);

extern imu_t imu;
#endif