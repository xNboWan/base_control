#include "bsp_can.h"
#include "can.h"

bool canStart(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef can_filter_config = {.FilterBank = 0,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_32BIT,
        .FilterIdHigh = 0x0000,
        .FilterIdLow = 0x0000,
        .FilterMaskIdHigh = 0x0000,
        .FilterMaskIdLow = 0x0000,
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .FilterActivation = CAN_FILTER_ENABLE,
        .SlaveStartFilterBank = 14};

    HAL_StatusTypeDef pass = HAL_OK;

    pass &= HAL_CAN_ConfigFilter(hcan, &can_filter_config);
    pass &= HAL_CAN_Start(hcan);
    pass &= HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    if (pass == HAL_OK)
        return true;
    else
        return false;
}

bool canSend(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *pdata, uint8_t len)
{
    uint32_t tx_mailbox;

    CAN_TxHeaderTypeDef tx_header = {.StdId = id,
        .ExtId = 0x00,
        .RTR = CAN_RTR_DATA,
        .DLC = len,
        .TransmitGlobalTime = DISABLE};

    if (HAL_CAN_AddTxMessage(hcan, &tx_header, pdata, &tx_mailbox))
        return true;
    else
        return false;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1)
        return;

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        CAN_RxHeaderTypeDef rx;
        uint8_t rx_buf[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, rx_buf) != HAL_OK)
            break;
    }
}