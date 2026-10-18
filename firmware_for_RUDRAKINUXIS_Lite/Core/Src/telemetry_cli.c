/**
 ******************************************************************************
 * @file           : telemetry_cli.c
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

#include "telemetry_cli.h"
#include "board_config.h"
#include "bootloader_jump.h"
#include "foc_core.h"
#include "safety_supervisor.h"
#include <stdarg.h>
#include <stdlib.h>

/* Global CLI Instance */
telemetry_cli_t g_cli;

/**
 * @brief  Initializes the Telemetry CLI and configures non-blocking DMA.
 */
void Telemetry_CLI_Init(void) {
  memset(&g_cli, 0, sizeof(telemetry_cli_t));
  g_cli.stream_mode = STREAM_MODE_CSV_SIMPLEFOC;
  g_cli.stream_interval_ms = 20U; /* 50 Hz telemetry stream */
  g_cli.last_stream_tick_ms = HAL_GetTick();

  /* Enable DMA1 & DMAMUX clocks */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);

  /*
   * Configure DMA1 Channel 2 for USART1 TX:
   * Memory to peripheral, non-increment peripheral, increment memory,
   * 8-bit data alignment.
   */
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_2,
                        LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_MODE_NORMAL |
                            LL_DMA_PERIPH_NOINCREMENT |
                            LL_DMA_MEMORY_INCREMENT | LL_DMA_PDATAALIGN_BYTE |
                            LL_DMA_MDATAALIGN_BYTE | LL_DMA_PRIORITY_LOW);

  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&(USART1->TDR));

  /* Route USART1 TX request to DMA1 Channel 2 via DMAMUX */
  LL_DMAMUX_SetRequestID(DMAMUX1, LL_DMAMUX_CHANNEL_1, LL_DMAMUX_REQ_USART1_TX);

  /* Enable USART1 TX DMA request */
  LL_USART_EnableDMAReq_TX(USART1);
}

/**
 * @brief  Transmits a formatted string over USART1 via DMA (non-blocking).
 */
bool Telemetry_CLI_Printf(const char *format, ...) {
  /* Check if previous DMA transfer is still running */
  if (LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_2) != 0U) {
    if (LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_2) > 0U) {
      return false; /* Busy */
    }
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
  }

  va_list args;
  va_start(args, format);
  int len = vsnprintf(g_cli.tx_buffer, sizeof(g_cli.tx_buffer), format, args);
  va_end(args);

  if (len <= 0) {
    return false;
  }

  /* Configure and trigger DMA transfer */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)g_cli.tx_buffer);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, (uint32_t)len);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

  return true;
}

/**
 * @brief  Streams high-rate CSV telemetry matching SimpleFOC Studio format:
 *         "I_q, I_d, V_bus, RPM, Temp\r\n"
 */
void Telemetry_CLI_Stream_Step(void) {
  if (g_cli.stream_mode == STREAM_MODE_OFF) {
    return;
  }

  uint32_t now = HAL_GetTick();
  if ((now - g_cli.last_stream_tick_ms) >= g_cli.stream_interval_ms) {
    g_cli.last_stream_tick_ms = now;

    if (g_cli.stream_mode == STREAM_MODE_CSV_SIMPLEFOC) {
      /* Format compatible with SimpleFOC Studio and Arduino Serial Plotter */
      Telemetry_CLI_Printf("%.2f,%.2f,%.2f,%.1f,%.1f\r\n", g_foc.i_dq.q,
                           g_foc.i_dq.d, g_foc.v_bus, g_foc.smo.rpm_mech,
                           g_safety.measured_temp_c);
    }
  }
}

/**
 * @brief  Executes a parsed CLI text command.
 */
static void Telemetry_CLI_Execute_Command(const char *cmd) {
  if (strcmp(cmd, "HELP") == 0) {
    Telemetry_CLI_Printf(
        "\r\n=== RUDRAKINUXIS-Lite FOC ESC CLI ===\r\n"
        "STATUS         - Print system telemetry & state\r\n"
        "ARM            - Arm ESC for motor control\r\n"
        "DISARM         - Disarm inverter & clear gates\r\n"
        "SET_IQ <val>   - Set commanded Iq current in Amperes\r\n"
        "CLEAR          - Clear latching safety faults\r\n"
        "STREAM <0|1>   - Toggle SimpleFOC telemetry stream\r\n"
        "REBOOT_BL      - Jump to ST Factory ROM Bootloader\r\n"
        "======================================\r\n");
  } else if (strcmp(cmd, "STATUS") == 0) {
    const char *state_str = "UNKNOWN";
    switch (g_foc.state) {
    case FOC_STATE_CALIBRATION:
      state_str = "CALIBRATION";
      break;
    case FOC_STATE_DISARMED:
      state_str = "DISARMED";
      break;
    case FOC_STATE_ARMED:
      state_str = "ARMED";
      break;
    case FOC_STATE_ALIGN:
      state_str = "ALIGN";
      break;
    case FOC_STATE_STARTUP_RAMP:
      state_str = "RAMP";
      break;
    case FOC_STATE_CLOSED_LOOP:
      state_str = "CLOSED_LOOP";
      break;
    case FOC_STATE_FAULT:
      state_str = "FAULT";
      break;
    default:
      break;
    }

    Telemetry_CLI_Printf("\r\n--- STATUS REPORT ---\r\n"
                         "State:       %s\r\n"
                         "V_Bus:       %.2f V\r\n"
                         "I_q:         %.2f A (Target: %.2f A)\r\n"
                         "I_d:         %.2f A\r\n"
                         "Mech RPM:    %.1f\r\n"
                         "Elec RPM:    %.1f\r\n"
                         "Temp:        %.1f C\r\n"
                         "Fault Flags: 0x%04X\r\n"
                         "ISR Time:    %.2f us (Max: %lu cycles)\r\n"
                         "---------------------\r\n",
                         state_str, g_foc.v_bus, g_foc.i_dq.q,
                         g_foc.target_iq_a, g_foc.i_dq.d, g_foc.smo.rpm_mech,
                         g_foc.smo.rpm_elec, g_safety.measured_temp_c,
                         (unsigned int)g_safety.active_faults,
                         g_foc.isr_duration_us, g_foc.isr_cycles_max);
  } else if (strcmp(cmd, "ARM") == 0) {
    FOC_Core_Arm(&g_foc);
    Telemetry_CLI_Printf("ESC ARMED\r\n");
  } else if (strcmp(cmd, "DISARM") == 0) {
    FOC_Core_Disarm(&g_foc);
    Telemetry_CLI_Printf("ESC DISARMED\r\n");
  } else if (strncmp(cmd, "SET_IQ ", 7) == 0) {
    float target = (float)atof(&cmd[7]);
    FOC_Core_SetTargetCurrent(&g_foc, target);
    Telemetry_CLI_Printf("Target Iq set to %.2f A\r\n", target);
  } else if (strcmp(cmd, "CLEAR") == 0) {
    if (Safety_ClearFaults()) {
      Telemetry_CLI_Printf("Faults cleared successfully.\r\n");
    } else {
      Telemetry_CLI_Printf("Unable to clear faults: condition persists.\r\n");
    }
  } else if (strcmp(cmd, "STREAM 0") == 0) {
    g_cli.stream_mode = STREAM_MODE_OFF;
    Telemetry_CLI_Printf("Telemetry stream paused.\r\n");
  } else if (strcmp(cmd, "STREAM 1") == 0) {
    g_cli.stream_mode = STREAM_MODE_CSV_SIMPLEFOC;
    Telemetry_CLI_Printf("Telemetry stream active.\r\n");
  } else if (strcmp(cmd, "REBOOT_BL") == 0) {
    Telemetry_CLI_Printf("Jumping to ST ROM Bootloader via 0xDEADBEEF...\r\n");

    /* Brief delay for UART flush */
    for (volatile uint32_t i = 0; i < 200000; i++) {
    }

    /* Execute soft reset with magic key in Backup SRAM */
    Bootloader_RequestJump();
  } else {
    Telemetry_CLI_Printf("Unknown command. Type 'HELP' for options.\r\n");
  }
}

/**
 * @brief  Polls the non-blocking UART receiver and executes parsed CLI
 * commands.
 */
void Telemetry_CLI_Process(void) {
  /* Read all available bytes from USART1 RX register */
  while (LL_USART_IsActiveFlag_RXNE(USART1) != 0U) {
    char c = (char)LL_USART_ReceiveData8(USART1);

    /* Carriage return or newline triggers command execution */
    if ((c == '\r') || (c == '\n')) {
      if (g_cli.cmd_index > 0U) {
        g_cli.cmd_buffer[g_cli.cmd_index] = '\0';
        Telemetry_CLI_Execute_Command(g_cli.cmd_buffer);
        g_cli.cmd_index = 0U;
      }
    } else if (c == '\b') {
      /* Backspace handling */
      if (g_cli.cmd_index > 0U) {
        g_cli.cmd_index--;
      }
    } else {
      if (g_cli.cmd_index < (CLI_CMD_MAX_LEN - 1U)) {
        g_cli.cmd_buffer[g_cli.cmd_index++] = c;
      }
    }
  }
}
