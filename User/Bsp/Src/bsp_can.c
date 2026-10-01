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

    if (HAL_CAN_ConfigFilter(hcan, &can_filter_config) != HAL_OK)
        return false;

    if (HAL_CAN_Start(hcan) != HAL_OK)
        return false;

    return HAL_CAN_ActivateNotification(
    hcan, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK;
}

bool canSend(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *pdata, uint8_t len)
{
    uint32_t tx_mailbox;

    CAN_TxHeaderTypeDef tx_header = {.StdId = id,
        .ExtId = 0x00,
        .RTR = CAN_RTR_DATA,
        .DLC = len,
        .TransmitGlobalTime = DISABLE};

    return HAL_CAN_AddTxMessage(hcan, &tx_header, pdata, &tx_mailbox) == HAL_OK;
}

