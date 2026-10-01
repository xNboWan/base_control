#include "debug.h"
#include "static_mem.h"
#include "imu.h"
#include "common.h"
#include "stdio.h"
#include "motor.h"

static TaskHandle_t debug_task_handle;

static void debugTask(void *arg);

static float acc[3];
static float gyro[3];
static float yaw;

bool debugInit(void)
{
    STATIC_MEM_TASK_ALLOC(debugTask, 256);
    debug_task_handle = STATIC_MEM_TASK_CREATE(debugTask, debugTask, "DEBUG", NULL, 1);
    if (debug_task_handle != NULL)
        return true;
    else
        return false;
}

void debugTask(void *arg)
{
    (void)arg;
    imuData_t imu_data;
    motorData_t motor_data[4];
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

        motorRead(&motor[FL], &motor_data[FL]);
        motorRead(&motor[FR], &motor_data[FR]);
        motorRead(&motor[RL], &motor_data[RL]);
        motorRead(&motor[RR], &motor_data[RR]);
        // printf("%f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", 
        //     acc[0], 
        //     acc[1],
        //     acc[2], 
        //     gyro[0], 
        //     gyro[1], 
        //     gyro[2], 
        //     yaw);

        // printf("%f, %f, %f, %d,%f, %f, %f, %d,%f, %f, %f, %d,%f, %f, %f, %d\n", 
        //     motor_data[FL].omega, 
        //     motor_data[FL].d_omega,
        //     motor_data[FL].lq,
        //     motor_data[FL].temperature,
        //     motor_data[FR].omega, 
        //     motor_data[FR].d_omega,
        //     motor_data[FR].lq,
        //     motor_data[FR].temperature,            
        //     motor_data[RL].omega, 
        //     motor_data[RL].d_omega,
        //     motor_data[RL].lq,
        //     motor_data[RL].temperature,           
        //     motor_data[RR].omega, 
        //     motor_data[RR].d_omega,
        //     motor_data[RR].lq,
        //     motor_data[RR].temperature);
        printf("%f, %f, %f, %d\n", 
            motor_data[RR].omega, 
            motor_data[RR].d_omega,
            motor_data[RR].lq,
            motor_data[RR].temperature);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}