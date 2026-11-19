/**
  ******************************************************************************
  * @file           : dshot_decoder.h
  * @brief          : High-speed DShot300 / DShot600 DMA frame decoder with CRC
  *                   verification, bidirectional telemetry, and RC PWM fallback.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 RUDRAKINUXIS Project.
  * All rights reserved.
  *
  ******************************************************************************
  */

#ifndef __DSHOT_DECODER_H
#define __DSHOT_DECODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* DShot Protocol Constants */
#define DSHOT_FRAME_BITS                (16U)
#define DSHOT_DMA_BUFFER_SIZE           (32U)

/* Command Thresholds */
#define DSHOT_CMD_DISARM                (0U)
#define DSHOT_CMD_MIN_THROTTLE          (48U)
#define DSHOT_CMD_MAX_THROTTLE          (2047U)

/* Standard RC PWM Constants (Microseconds) */
#define PWM_PULSE_MIN_US                (950U)
#define PWM_PULSE_ZERO_US               (1000U)
#define PWM_PULSE_MAX_US                (2000U)

/* Protocol Modes */
typedef enum
{
    PROTOCOL_DETECTING = 0,
    PROTOCOL_DSHOT300,
    PROTOCOL_DSHOT600,
    PROTOCOL_STANDARD_PWM
} throttle_protocol_t;

/* DShot Telemetry Return Frame Structure */
typedef struct
{
    uint32_t erpm;
    uint16_t voltage_cval;  /**< 0.01V units */
    uint16_t current_cval;  /**< 0.01A units */
    uint8_t  temperature_c; /**< Degrees Celsius */
} dshot_telemetry_t;

/* Master DShot Decoder State Structure */
typedef struct
{
    throttle_protocol_t protocol;
    volatile uint16_t dma_buffer[DSHOT_DMA_BUFFER_SIZE];

    /* Decoded Outputs */
    uint16_t raw_throttle;          /**< Raw 11-bit command (0 to 2047) */
    float    normalized_throttle;   /**< 0.0f (0%) to 1.0f (100%) */
    bool     telemetry_requested;   /**< Telemetry bit set */
    bool     frame_valid;           /**< Passed CRC check */
    bool     bidirectional_mode;    /**< Inverted CRC telemetry mode */

    /* Statistics & Health */
    uint32_t packets_received;
    uint32_t packets_crc_errors;
    uint32_t last_packet_time_ms;

    /* Telemetry Return Packet */
    dshot_telemetry_t telemetry_data;
} dshot_decoder_t;

/* Global Decoder Instance */
extern dshot_decoder_t g_dshot;

/*============================================================================*/
/*                      FUNCTION PROTOTYPES                                   */
/*============================================================================*/

/**
  * @brief  Initializes the DShot timer (TIM2) and DMA stream on PA15.
  * @retval None
  */
void DShot_Decoder_Init(void);

/**
  * @brief  Processes incoming DMA pulses, decodes 16-bit DShot packet,
  *         validates 4-bit CRC, and updates throttle setpoints.
  * @retval true if a valid new packet was decoded, false otherwise.
  */
bool DShot_Decoder_Process(void);

/**
  * @brief  Encodes electrical RPM into 16-bit GCR return frame for bidirectional DShot.
  * @param  erpm Electrical RPM of the motor
  * @retval 16-bit encoded response
  */
uint16_t DShot_Encode_ERPM(uint32_t erpm);

/**
  * @brief  Resets the decoder and packet statistics.
  * @retval None
  */
void DShot_Decoder_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __DSHOT_DECODER_H */
