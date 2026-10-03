/* Host-only HAL/RTOS substitutes for testing the real remote/chassis sources. */
#ifndef FIRMWARE_TEST_STUBS_H
#define FIRMWARE_TEST_STUBS_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "pid.h"

#define PI 3.14159265358979323846f
#define FORWARD 1
#define REVERSE -1
enum { FL, FR, RL, RR };
typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef void *TaskHandle_t;
typedef void *QueueHandle_t;
typedef void *StreamBufferHandle_t;
typedef struct { int unused; } StaticStreamBuffer_t;
#define pdFALSE 0
#define pdTRUE 1
#define portMAX_DELAY UINT32_MAX
#define HAL_MAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define portYIELD_FROM_ISR(woken) ((void)(woken))
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define STATIC_MEM_TASK_ALLOC(name, size) ((void)0)
#define STATIC_MEM_TASK_CREATE(...) ((TaskHandle_t)1)
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *last, TickType_t period);

typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } CRC_HandleTypeDef;
extern UART_HandleTypeDef huart8;
extern CRC_HandleTypeDef hcrc;
enum { UART8_IRQn = 8, HAL_OK = 0 };
#define HAL_NVIC_SetPriority(...) ((void)0)
#define HAL_NVIC_EnableIRQ(...) ((void)0)
#define __HAL_UART_CLEAR_OREFLAG(...) ((void)0)
static inline int HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *buf, unsigned length)
{ (void)uart; (void)buf; (void)length; return HAL_OK; }
static inline int HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{ (void)uart; return HAL_OK; }
uint32_t HAL_CRC_Calculate(CRC_HandleTypeDef *handle, uint32_t *words, uint32_t count);
static inline StreamBufferHandle_t xStreamBufferCreateStatic(size_t size, size_t trigger,
                                                           uint8_t *storage, StaticStreamBuffer_t *control)
{ (void)size; (void)trigger; (void)control; return storage; }
static inline void vStreamBufferDelete(StreamBufferHandle_t stream) { (void)stream; }
static inline size_t xStreamBufferSendFromISR(StreamBufferHandle_t stream, const void *data,
                                            size_t size, BaseType_t *woken)
{ (void)stream; (void)data; (void)woken; return size; }
size_t xStreamBufferReceive(StreamBufferHandle_t stream, void *buf, size_t size, TickType_t timeout);

typedef struct { int unused; } CAN_HandleTypeDef;
extern CAN_HandleTypeDef hcan1;
typedef struct { int unused; } motorOps_t;
typedef struct { CAN_HandleTypeDef *hcan; int id; } m3508Ctx_t;
extern motorOps_t m3508_ops;
typedef struct { void *ctx; const motorOps_t *ops; } motor_t;
typedef struct { int unused; } wheel_t;
typedef struct { wheel_t *wheel; unsigned wheel_num; } wheelGroup_t;
typedef struct { float d_theta; } wheelCmd_t;
typedef struct {
    motor_t motor;
    float wheel_radius, gear_ratio;
    int dir;
    pidCfg_t pid_cfg;
    float v_max;
} wheelCfg_t;
static inline void wheelInit(wheel_t *wheel, const wheelCfg_t *cfg) { (void)wheel; (void)cfg; }
void wheelGroupWriteAndSend(wheelGroup_t *group, const wheelCmd_t *cmd, float dt);
#include "imu.h"
#endif
