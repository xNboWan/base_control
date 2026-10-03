/**
 * @file debug.c
 * @author 李嘉羽 (aa01082241015@gmail.com)
 * @brief 调试模块实现
 * @version 0.1
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "debug.h"
#include "static_mem.h"
#include "imu.h"
#include "generic_def.h"
#include "stdio.h"
#include "motor.h"
#include "wheel.h"

static TaskHandle_t debug_task_handle;

static void debugTask(void *arg);

static float acc[3];
static float gyro[3];
static float yaw;

static bool is_init = false;

bool debugInit(void)
{
    if (is_init) return true;

    STATIC_MEM_TASK_ALLOC(debugTask, 512);
    debug_task_handle = STATIC_MEM_TASK_CREATE(debugTask, debugTask, "DEBUG", NULL, 1);
    if (!debug_task_handle)
        return false;

    is_init = true;
    return true;
}

void debugTask(void *arg)
{
    (void)arg;
    imuData_t imu_data;
    wheelData_t wheel_data;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    for (;;)
    {
        imuRead(&imu, &imu_data);
        for (uint8_t i = 0; i < 3; i++)
        {
            acc[i] = imu_data.accel[i];
            gyro[i] = imu_data.gyro[i];
        }
        yaw = imu_data.yaw;

        printf("%f, %.3f, %.3f, %.3f\n", 
            acc[0], 
            acc[1],
            acc[2],  
            yaw);

        
            // printf("%f, %f, %f, %d,%f, %f, %f, %d,%f, %f, %f, %d,%f, %f, %f, %d\n", 
            //     motor_data[FL].theta, 
            //     motor_data[FL].d_theta,
            //     motor_data[FL].lq,
            //     motor_data[FL].temperature,
            //     motor_data[FR].theta, 
            //     motor_data[FR].d_theta,
            //     motor_data[FR].lq,
            //     motor_data[FR].temperature,            
            //     motor_data[RL].theta, 
            //     motor_data[RL].d_theta,
            //     motor_data[RL].lq,
            //     motor_data[RL].temperature,           
            //     motor_data[RR].theta, 
            //     motor_data[RR].d_theta,
            //     motor_data[RR].lq,
            //     motor_data[RR].temperature);
            // printf("%f, %f, %f, %d\n", 
            //     motor_data[RR].theta, 
            //     motor_data[RR].d_theta,
            //     motor_data[RR].lq,
            //     motor_data[RR].temperature);
        wheelRead(&wheel[RL], &wheel_data);

        //printf("%f, %f, %f\n", wheel_data.d_theta, wheel_data.target_d_theta, wheel_data.v);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}