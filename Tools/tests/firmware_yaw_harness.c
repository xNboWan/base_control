/* Compile unchanged production bodies with host HAL/RTOS substitutes. */
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include "firmware_stubs.h"
#define is_init chassis_test_is_init
#include "../../User/Modules/Src/chassis.c"
#undef is_init
#define is_init remote_test_is_init
#include "../../User/Modules/Src/remote.c"
#undef is_init

UART_HandleTypeDef huart8;
CRC_HandleTypeDef hcrc;
CAN_HandleTypeDef hcan1;
motorOps_t m3508_ops;
imu_t imu;
static uint8_t input[4096];
static size_t input_size, input_pos;
static float measured_yaw;
static bool imu_ready;
static wheelCmd_t sent_wheels[4];
static jmp_buf task_exit;
static unsigned task_cycles, requested_cycles = 1;
static float rotations[32];

TickType_t xTaskGetTickCount(void) { return 0; }
void vTaskDelayUntil(TickType_t *last, TickType_t period)
{
    (void)last; (void)period;
    rotations[task_cycles++] = (sent_wheels[FR].d_theta - sent_wheels[FL].d_theta) * WHEEL_RADIUS / (2.0f * (Rx + Ry));
    if (task_cycles >= requested_cycles) longjmp(task_exit, 1);
}
size_t xStreamBufferReceive(StreamBufferHandle_t stream, void *buf, size_t size, TickType_t timeout)
{
    (void)stream; (void)timeout;
    if (!size || input_pos == input_size) return 0;
    *(uint8_t *)buf = input[input_pos++];
    return 1;
}
uint32_t HAL_CRC_Calculate(CRC_HandleTypeDef *handle, uint32_t *words, uint32_t count)
{
    (void)handle;
    uint32_t crc = UINT32_MAX;
    for (unsigned word = 0; word < count; ++word)
        for (int bit = 31; bit >= 0; --bit)
        {
            bool feedback = ((crc >> 31) ^ (words[word] >> bit)) & 1u;
            crc <<= 1;
            if (feedback) crc ^= 0x04c11db7u;
        }
    return crc;
}
bool imuRead(imu_t *device, imuData_t *data)
{
    (void)device;
    if (!imu_ready) return false;
    *data = (imuData_t){.yaw = measured_yaw};
    return true;
}
void wheelGroupWriteAndSend(wheelGroup_t *group, const wheelCmd_t *cmd, float dt)
{ (void)group; (void)dt; memcpy(sent_wheels, cmd, sizeof(sent_wheels)); }

int main(int argc, char **argv)
{
    if (!chassisInit() || !remoteInit()) return 2;
    if (argc == 2 && strcmp(argv[1], "config") == 0)
    {
        printf("%.9g %.9g %.9g %.9g\n", (double)yaw_pid.cfg.kp, (double)yaw_pid.cfg.ki,
               (double)yaw_pid.cfg.out_min, (double)yaw_pid.cfg.out_max);
        return 0;
    }
    if (argc > 1)
    {
        if (argc != 6 && argc != 7) return 2;
        if (argc == 7)
        {
            requested_cycles = strtoul(argv[6], NULL, 10);
            if (requested_cycles == 0 || requested_cycles > 32) return 2;
        }
        chassisCmd_t cmd = {.mode = atoi(argv[1]), .yaw = strtof(argv[2], NULL),
                           .d_yaw = strtof(argv[3], NULL)};
        measured_yaw = strtof(argv[4], NULL);
        imu_ready = atoi(argv[5]) != 0;
        if (!chassisSetCommand(&cmd)) { puts("rejected"); return 0; }
        if (setjmp(task_exit) == 0) chassisTask(NULL);
        for (unsigned cycle = 0; cycle < task_cycles; ++cycle)
            printf("%.9g\n", (double)rotations[cycle]);
        return 0;
    }
    input_size = fread(input, 1, sizeof(input), stdin);
    uint8_t frame[27];
    uint16_t size;
    while ((size = remoteRecvFrame(frame, sizeof(frame), 20)) != 0)
        if (remoteParseFrame(frame, size))
            printf("%d %.9g %.9g %.9g %.9g\n", chassis_cmd.mode,
                   (double)chassis_cmd.vx, (double)chassis_cmd.vy,
                   (double)chassis_cmd.yaw, (double)chassis_cmd.d_yaw);
    return 0;
}
