/**
  ******************************************************************************
  * @file           : canfd_interface.c
  * @brief          : High-speed CAN FD / DroneCAN communication interface.
  *                   1 Mbps nominal / 5 Mbps data bit rate with BRS,
  *                   command parsing, and high-rate telemetry broadcast.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 RUDRAKINUXIS Project.
  * All rights reserved.
  *
  ******************************************************************************
  */

#include "canfd_interface.h"

/* Global CAN FD Controller Instance */
canfd_interface_t g_can;

/**
  * @brief  Initializes FDCAN1 in CAN FD mode (1 Mbps nominal, 5 Mbps data bit rate).
  *         Calculates exact bit timings for 170 MHz peripheral clock.
  */
HAL_StatusTypeDef CANFD_Init(void)
{
    memset(&g_can, 0, sizeof(canfd_interface_t));

    g_can.hfdcan.Instance                  = FDCAN1;
    g_can.hfdcan.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    g_can.hfdcan.Init.FrameFormat          = FDCAN_FRAME_FD_BRS; /* CAN FD with Bit Rate Switching */
    g_can.hfdcan.Init.Mode                 = FDCAN_MODE_NORMAL;
    g_can.hfdcan.Init.AutoRetransmission   = ENABLE;
    g_can.hfdcan.Init.TransmitPause        = DISABLE;
    g_can.hfdcan.Init.ProtocolException    = DISABLE;

    /*
     * Nominal Bit Timing: 1.0 Mbps
     * Clock = 170 MHz
     * Prescaler = 10 -> Time Quantum = 17 MHz
     * Bit Period = 17 tq -> (1 + 12 + 4) -> 17 MHz / 17 = 1.0 Mbps
     * Sample Point = (1 + 12) / 17 = 76.47%
     */
    g_can.hfdcan.Init.NominalPrescaler     = 10U;
    g_can.hfdcan.Init.NominalSyncJumpWidth = 4U;
    g_can.hfdcan.Init.NominalTimeSeg1      = 12U;
    g_can.hfdcan.Init.NominalTimeSeg2      = 4U;

    /*
     * Data Bit Timing: 5.0 Mbps
     * Clock = 170 MHz
     * Prescaler = 2 -> Time Quantum = 85 MHz
     * Bit Period = 17 tq -> (1 + 12 + 4) -> 85 MHz / 17 = 5.0 Mbps
     * Sample Point = (1 + 12) / 17 = 76.47%
     */
    g_can.hfdcan.Init.DataPrescaler        = 2U;
    g_can.hfdcan.Init.DataSyncJumpWidth    = 4U;
    g_can.hfdcan.Init.DataTimeSeg1         = 12U;
    g_can.hfdcan.Init.DataTimeSeg2         = 4U;

    /* Message RAM Configuration */
    g_can.hfdcan.Init.StdFiltersNbr        = 1U;
    g_can.hfdcan.Init.ExtFiltersNbr        = 0U;
    g_can.hfdcan.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

    HAL_StatusTypeDef status = HAL_FDCAN_Init(&g_can.hfdcan);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Configure Standard Filter to route throttle commands (0x200) into RX FIFO 0 */
    FDCAN_FilterTypeDef filter_config = {0};
    filter_config.IdType       = FDCAN_STANDARD_ID;
    filter_config.FilterIndex  = 0U;
    filter_config.FilterType   = FDCAN_FILTER_RANGE;
    filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter_config.FilterID1    = CAN_ID_THROTTLE_CMD;
    filter_config.FilterID2    = CAN_ID_CONFIG_CMD;

    HAL_FDCAN_ConfigFilter(&g_can.hfdcan, &filter_config);

    /* Reject non-matching frames */
    HAL_FDCAN_ConfigGlobalFilter(&g_can.hfdcan,
                                 FDCAN_REJECT,
                                 FDCAN_REJECT,
                                 FDCAN_FILTER_REMOTE,
                                 FDCAN_FILTER_REMOTE);

    /* Start the FDCAN controller */
    status = HAL_FDCAN_Start(&g_can.hfdcan);
    if (status == HAL_OK)
    {
        g_can.is_online = true;
    }

    return status;
}

/**
  * @brief  Transmits high-rate telemetry frame (voltage, current, RPM, temp, status).
  */
HAL_StatusTypeDef CANFD_TransmitTelemetry(float vbus_v,
                                         float current_a,
                                         float rpm,
                                         float temp_c,
                                         uint8_t status)
{
    if (!g_can.is_online)
    {
        return HAL_ERROR;
    }

    FDCAN_TxHeaderTypeDef tx_header = {0};
    tx_header.Identifier          = CAN_ID_TELEMETRY_TX;
    tx_header.IdType              = FDCAN_STANDARD_ID;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.DataLength          = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_ON;   /* 5 Mbps data payload */
    tx_header.FDFormat            = FDCAN_FD_CAN;   /* CAN FD Frame */
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0U;

    can_telemetry_payload_t payload;
    payload.vbus_mv       = (uint16_t)(vbus_v * 1000.0f);
    payload.current_ca    = (int16_t)(current_a * 100.0f);
    payload.rpm_mech      = (int16_t)rpm;
    payload.temperature_c = (int8_t)temp_c;
    payload.status_flags  = status;

    HAL_StatusTypeDef res = HAL_FDCAN_AddMessageToTxFifoQ(&g_can.hfdcan,
                                                         &tx_header,
                                                         (uint8_t *)&payload);
    if (res == HAL_OK)
    {
        g_can.tx_packet_count++;
    }

    return res;
}

/**
  * @brief  Checks and processes incoming CAN FD messages in RX FIFO 0.
  */
bool CANFD_Process_Rx(void)
{
    if (!g_can.is_online)
    {
        return false;
    }

    if (HAL_FDCAN_GetRxFifoFillLevel(&g_can.hfdcan, FDCAN_RX_FIFO0) == 0U)
    {
        return false;
    }

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64] = {0};

    HAL_StatusTypeDef res = HAL_FDCAN_GetRxMessage(&g_can.hfdcan,
                                                   FDCAN_RX_FIFO0,
                                                   &rx_header,
                                                   rx_data);
    if (res != HAL_OK)
    {
        return false;
    }

    g_can.rx_packet_count++;
    g_can.last_rx_time_ms = HAL_GetTick();

    /* Parse Actuator Throttle Command */
    if (rx_header.Identifier == CAN_ID_THROTTLE_CMD)
    {
        /*
         * Packet Layout:
         * Byte [0..1]: uint16_t throttle_command (0 to 10000 -> 0.0% to 100.0%)
         * Byte [2]:    uint8_t  arm_command (0 = Disarm, 1 = Arm)
         */
        uint16_t cmd_raw = (uint16_t)(rx_data[0] | (rx_data[1] << 8U));
        uint8_t  arm_cmd = rx_data[2];

        if (cmd_raw > 10000U)
        {
            cmd_raw = 10000U;
        }

        g_can.target_throttle = (float)cmd_raw / 10000.0f;
        g_can.arm_commanded   = (arm_cmd == 1U);

        return true;
    }

    return false;
}
