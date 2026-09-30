#include "stdbool.h"

#include "motor.h"
#include "m3508.h"
#include "bsp_can.h"

#define SEND_MSG_LEN 8
#define IDENTIFIER1  0x200
#define IDENTIFIER2  0x1FF

static bool m3508Init(void *ctx);
static bool m3508Read(void *ctx, motorData_t *pdata);
static bool m3508Write(void *ctx, motorCmd_t *pdata);

motorOps_t m3508_ops = {.init = m3508Init, .read = m3508Read, .write = m3508Write};

bool m3508Init(void *ctx)
{
    m3508Ctx_t *m3508_ctx = ctx;
    return canStart(&m3508_ctx->hcan);
}

static bool m3508Read(void *ctx, motorData_t *pdata)
{
    m3508Ctx_t *m3508_ctx = ctx;
}

static bool m3508Write(void *ctx, motorCmd_t *pdata)
{
    m3508Ctx_t *m3508_ctx = ctx;
    uint16_t id = 1;
    uint8_t torque[8] = {0};

    if (m3508_ctx->id < 5)
    {
        id = IDENTIFIER1;
        switch (m3508_ctx->id) 
        {
            case 1: torque[0] = pdata->torque & 0xFF; break;
            torque[1] = (pdata->torque >> 8) & 0xFF; break;
            case 2: torque[2] = pdata->torque & 0xFF; break;
            torque[3] = (pdata->torque >> 8) & 0xFF; break;
            case 3: torque[4] = pdata->torque & 0xFF; break;
            torque[5] = (pdata->torque >> 8) & 0xFF; break;
            case 4: torque[6] = pdata->torque & 0xFF; break;
            torque[7] = (pdata->torque >> 8) & 0xFF; break;
            default: break;
        }
    }
    else
    {
        id = IDENTIFIER2;
        switch (m3508_ctx->id) 
        {
            case 5: torque[0] = pdata->torque & 0xFF; break;
                torque[1] = (pdata->torque >> 8) & 0xFF; break;
            case 6: torque[2] = pdata->torque & 0xFF; break;
                torque[3] = (pdata->torque >> 8) & 0xFF; break;
            case 7: torque[4] = pdata->torque & 0xFF; break;
                torque[5] = (pdata->torque >> 8) & 0xFF; break;
            case 8: torque[6] = pdata->torque & 0xFF; break;
                torque[7] = (pdata->torque >> 8) & 0xFF; break;
            default: break;
        }
    }
    return canSend(&m3508_ctx->hcan, id, torque, SEND_MSG_LEN);
}