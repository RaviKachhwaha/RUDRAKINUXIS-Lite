/**
 ******************************************************************************
 * @file           : canfd_interface.h
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

#ifndef __CANFD_INTERFACE_H
#define __CANFD_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Standard CAN Message IDs */
#define CAN_ID_THROTTLE_CMD (0x200U)
#define CAN_ID_TELEMETRY_TX (0x201U)
#define CAN_ID_CONFIG_CMD (0x202U)

/* CAN FD Status Bitfield Flags */
#define CAN_STATUS_ARMED_BIT (1U << 0)
#define CAN_STATUS_CLOSED_LOOP_BIT (1U << 1)
#define CAN_STATUS_FAULT_BIT (1U << 2)
#define CAN_STATUS_OCP_BIT (1U << 3)
#define CAN_STATUS_OVP_BIT (1U << 4)
#define CAN_STATUS_UVLO_BIT (1U << 5)
#define CAN_STATUS_OTP_BIT (1U << 6)
#define CAN_STATUS_STALL_BIT (1U << 7)

/* Telemetry Payload Structure */
typedef struct __attribute__((packed)) {
  uint16_t vbus_mv;     /**< Bus Voltage in mV (e.g. 25200 = 25.20V) */
  int16_t current_ca;   /**< Bus Current in cA (e.g. 2700 = 27.00A) */
  int16_t rpm_mech;     /**< Mechanical RPM */
  int8_t temperature_c; /**< Board temperature (-40 to +125 C) */
  uint8_t status_flags; /**< Bitfield of status & fault states */
} can_telemetry_payload_t;

/* CAN Controller Handle */
typedef struct {
  FDCAN_HandleTypeDef hfdcan;
  uint32_t rx_packet_count;
  uint32_t tx_packet_count;
  uint32_t last_rx_time_ms;
  float target_throttle; /**< Decoded target throttle [0.0f, 1.0f] */
  bool arm_commanded;
  bool is_online;
} canfd_interface_t;

extern canfd_interface_t g_can;

/*============================================================================*/
/*                      FUNCTION PROTOTYPES                                   */
/*============================================================================*/

/**
 * @brief  Initializes FDCAN1 in CAN FD mode (1 Mbps nominal, 5 Mbps data bit
 * rate).
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef CANFD_Init(void);

/**
 * @brief  Transmits high-rate telemetry frame (voltage, current, RPM, temp,
 * status).
 * @param  vbus_v     DC Bus voltage in Volts
 * @param  current_a  DC Bus current in Amperes
 * @param  rpm        Mechanical RPM
 * @param  temp_c     Board temperature in Celsius
 * @param  status     Bitfield status flags
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef CANFD_TransmitTelemetry(float vbus_v, float current_a,
                                          float rpm, float temp_c,
                                          uint8_t status);

/**
 * @brief  Checks and processes incoming CAN FD messages in RX FIFO 0.
 * @retval true if a valid message was processed
 */
bool CANFD_Process_Rx(void);

#ifdef __cplusplus
}
#endif

#endif /* __CANFD_INTERFACE_H */
