/**
 ******************************************************************************
 * @file           : telemetry_cli.h
 * @brief          : Serial CLI & High-Rate Live Telemetry Streaming Interface.
 *                   Non-blocking circular DMA buffer on USART1 (CP2102N
 * bridge), SimpleFOC Studio CSV streaming, and ASCII CLI command parser.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 RUDRAKINUXIS Project.
 * All rights reserved.
 *
 ******************************************************************************
 */

#ifndef __TELEMETRY_CLI_H
#define __TELEMETRY_CLI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Buffer Sizes */
#define CLI_TX_BUFFER_SIZE (256U)
#define CLI_RX_BUFFER_SIZE (128U)
#define CLI_CMD_MAX_LEN (64U)

/* Stream Modes */
typedef enum {
  STREAM_MODE_OFF = 0,
  STREAM_MODE_CSV_SIMPLEFOC, /**< Iq, Id, Vbus, RPM, Temp */
  STREAM_MODE_VERBOSE        /**< Human readable debug status */
} telemetry_stream_mode_t;

/* Telemetry CLI State Structure */
typedef struct {
  telemetry_stream_mode_t stream_mode;
  uint32_t last_stream_tick_ms;
  uint32_t stream_interval_ms;

  /* DMA Buffers */
  char tx_buffer[CLI_TX_BUFFER_SIZE];
  char rx_buffer[CLI_RX_BUFFER_SIZE];
  uint16_t rx_index;

  /* Command Parsing */
  char cmd_buffer[CLI_CMD_MAX_LEN];
  uint8_t cmd_index;
  bool new_cmd_ready;
} telemetry_cli_t;

extern telemetry_cli_t g_cli;

/*============================================================================*/
/*                      FUNCTION PROTOTYPES                                   */
/*============================================================================*/

/**
 * @brief  Initializes the Telemetry CLI and configures non-blocking DMA.
 * @retval None
 */
void Telemetry_CLI_Init(void);

/**
 * @brief  Transmits a formatted string over USART1 via DMA (non-blocking).
 * @param  format Printf-style format string
 * @retval true if transmission started, false if transmitter is busy.
 */
bool Telemetry_CLI_Printf(const char *format, ...);

/**
 * @brief  Streams high-rate CSV telemetry matching SimpleFOC Studio format:
 *         "I_q, I_d, V_bus, RPM, Temp\r\n"
 * @retval None
 */
void Telemetry_CLI_Stream_Step(void);

/**
 * @brief  Polls the non-blocking UART receiver and executes parsed CLI
 * commands.
 * @retval None
 */
void Telemetry_CLI_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __TELEMETRY_CLI_H */
