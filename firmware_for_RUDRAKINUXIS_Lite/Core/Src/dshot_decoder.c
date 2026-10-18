/**
 ******************************************************************************
 * @file           : dshot_decoder.c
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

#include "dshot_decoder.h"
#include "board_config.h"

/* Global DShot Decoder Instance */
dshot_decoder_t g_dshot;

/* Pulse width thresholds at 170 MHz clock (1 tick = 5.882 ns) */
#define DSHOT300_BIT_THRESHOLD_TICKS (318U) /* 1.87 us */
#define DSHOT600_BIT_THRESHOLD_TICKS (159U) /* 0.93 us */
#define DSHOT_MIN_PULSE_TICKS (50U)         /* Glitch rejection (< 300 ns) */

/**
 * @brief  Initializes the DShot input capture timer (TIM2) and DMA stream.
 *         PA15 is connected to TIM2_CH1 (AF1).
 */
void DShot_Decoder_Init(void) {
  memset(&g_dshot, 0, sizeof(dshot_decoder_t));
  g_dshot.protocol = PROTOCOL_DETECTING;

  /* Enable TIM2 and DMA1 clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);

  /* Configure TIM2 Time Base: 170 MHz counting rate, 32-bit counter */
  LL_TIM_InitTypeDef tim_base = {0};
  tim_base.Prescaler = 0U;
  tim_base.CounterMode = LL_TIM_COUNTERMODE_UP;
  tim_base.Autoreload = 0xFFFFFFFFUL;
  tim_base.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  tim_base.RepetitionCounter = 0U;
  LL_TIM_Init(TIM2, &tim_base);

  /* Configure TIM2 Channel 1 for Input Capture (Both Edges or Pulse
   * Measurement) */
  LL_TIM_IC_InitTypeDef ic_init = {0};
  ic_init.ICPolarity = LL_TIM_IC_POLARITY_RISING;
  ic_init.ICActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI;
  ic_init.ICPrescaler = LL_TIM_ICPSC_DIV1;
  ic_init.ICFilter = LL_TIM_IC_FILTER_FDIV1_N2;
  LL_TIM_IC_Init(TIM2, LL_TIM_CHANNEL_CH1, &ic_init);

  /* Configure DMA1 Channel 5 for TIM2 CH1 capture transfers */
  LL_DMA_ConfigTransfer(
      DMA1, LL_DMA_CHANNEL_5,
      LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_MODE_CIRCULAR |
          LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
          LL_DMA_PDATAALIGN_HALFWORD | LL_DMA_MDATAALIGN_HALFWORD |
          LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&(TIM2->CCR1));
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)g_dshot.dma_buffer);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, DSHOT_DMA_BUFFER_SIZE);

  /* Map TIM2_CH1 request to DMA1 Channel 5 via DMAMUX */
  LL_DMAMUX_SetRequestID(DMAMUX1, LL_DMAMUX_CHANNEL_4, LL_DMAMUX_REQ_TIM2_CH1);

  /* Enable DMA Channel */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);

  /* Enable TIM2 Capture DMA Request */
  LL_TIM_EnableDMAReq_CC1(TIM2);

  /* Enable TIM2 Counter */
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableCounter(TIM2);
}

/**
 * @brief  Resets the decoder state and packet statistics.
 */
void DShot_Decoder_Reset(void) {
  g_dshot.raw_throttle = 0U;
  g_dshot.normalized_throttle = 0.0f;
  g_dshot.telemetry_requested = false;
  g_dshot.frame_valid = false;
}

/**
 * @brief  Calculates DShot 4-bit CRC checksum.
 * @param  frame 16-bit packet word
 * @param  inverted true for bidirectional DShot inverted CRC
 * @retval Computed 4-bit CRC
 */
static inline uint8_t DShot_Calculate_CRC(uint16_t frame, bool inverted) {
  uint16_t c_val = frame >> 4U;
  uint8_t crc = (c_val ^ (c_val >> 4U) ^ (c_val >> 8U)) & 0x0FU;
  if (inverted) {
    crc = (~crc) & 0x0FU;
  }
  return crc;
}

/**
 * @brief  Processes incoming DMA pulses, decodes 16-bit DShot packet,
 *         validates 4-bit CRC, and updates throttle setpoints.
 */
bool DShot_Decoder_Process(void) {
  /* Check how many samples have been moved by DMA */
  uint16_t dma_remaining = LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5);
  uint16_t captured_count = DSHOT_DMA_BUFFER_SIZE - dma_remaining;

  /* A complete DShot packet requires 16 bit-pulses */
  if (captured_count < DSHOT_FRAME_BITS) {
    return false;
  }

  uint16_t decoded_packet = 0U;
  uint32_t bit_threshold = DSHOT300_BIT_THRESHOLD_TICKS;

  /* Auto-detect protocol on first packet if detecting */
  if (g_dshot.protocol == PROTOCOL_DETECTING) {
    /* Check first pulse width */
    uint16_t first_pulse = g_dshot.dma_buffer[0];
    if (first_pulse > 200U) {
      g_dshot.protocol = PROTOCOL_DSHOT300;
      bit_threshold = DSHOT300_BIT_THRESHOLD_TICKS;
    } else {
      g_dshot.protocol = PROTOCOL_DSHOT600;
      bit_threshold = DSHOT600_BIT_THRESHOLD_TICKS;
    }
  } else if (g_dshot.protocol == PROTOCOL_DSHOT600) {
    bit_threshold = DSHOT600_BIT_THRESHOLD_TICKS;
  } else {
    bit_threshold = DSHOT300_BIT_THRESHOLD_TICKS;
  }

  /* Decode 16 bits sequentially */
  for (uint8_t i = 0U; i < DSHOT_FRAME_BITS; i++) {
    uint16_t pulse = g_dshot.dma_buffer[i];

    /* Glitch rejection */
    if (pulse < DSHOT_MIN_PULSE_TICKS) {
      g_dshot.packets_crc_errors++;
      return false;
    }

    decoded_packet <<= 1U;
    if (pulse > bit_threshold) {
      decoded_packet |= 1U;
    }
  }

  /* Extract components: [15..5] = Throttle (11b), [4] = Telemetry (1b), [3..0]
   * = CRC (4b) */
  uint8_t received_crc = (uint8_t)(decoded_packet & 0x0FU);
  uint8_t calc_crc_std = DShot_Calculate_CRC(decoded_packet, false);
  uint8_t calc_crc_inv = DShot_Calculate_CRC(decoded_packet, true);

  bool crc_ok = false;
  if (calc_crc_std == received_crc) {
    crc_ok = true;
    g_dshot.bidirectional_mode = false;
  } else if (calc_crc_inv == received_crc) {
    crc_ok = true;
    g_dshot.bidirectional_mode = true;
  }

  if (!crc_ok) {
    g_dshot.packets_crc_errors++;
    g_dshot.frame_valid = false;
    return false;
  }

  /* Valid Frame: Unpack throttle & telemetry bit */
  g_dshot.frame_valid = true;
  g_dshot.packets_received++;
  g_dshot.last_packet_time_ms = HAL_GetTick();

  uint16_t throttle_command = (decoded_packet >> 5U) & 0x07FFU;
  g_dshot.telemetry_requested = ((decoded_packet >> 4U) & 0x01U) != 0U;
  g_dshot.raw_throttle = throttle_command;

  /* Map throttle to normalized setpoint [0.0f, 1.0f] */
  if (throttle_command == DSHOT_CMD_DISARM) {
    g_dshot.normalized_throttle = 0.0f;
  } else if (throttle_command >= DSHOT_CMD_MIN_THROTTLE) {
    if (throttle_command > DSHOT_CMD_MAX_THROTTLE) {
      throttle_command = DSHOT_CMD_MAX_THROTTLE;
    }
    g_dshot.normalized_throttle =
        (float)(throttle_command - DSHOT_CMD_MIN_THROTTLE) /
        (float)(DSHOT_CMD_MAX_THROTTLE - DSHOT_CMD_MIN_THROTTLE);
  } else {
    /* Special commands (1..47) such as beep, direction change */
    g_dshot.normalized_throttle = 0.0f;
  }

  return true;
}

/**
 * @brief  Encodes electrical RPM into 16-bit GCR return frame for bidirectional
 * DShot. Format: 9-bit mantissa + 3-bit exponent + 4-bit inverted checksum.
 */
uint16_t DShot_Encode_ERPM(uint32_t erpm) {
  if (erpm == 0U) {
    return 0xFFFFU; /* Zero speed packet */
  }

  /* Convert ERPM to period in microseconds: T_us = (1,000,000 * 60) / erpm */
  uint32_t period_us = (60000000UL) / erpm;

  /* Compute 9-bit mantissa and 3-bit exponent */
  uint8_t exp = 0U;
  while ((period_us > 0x1FFU) && (exp < 7U)) {
    period_us >>= 1U;
    exp++;
  }

  if (period_us > 0x1FFU) {
    period_us = 0x1FFU;
  }

  uint16_t value = (uint16_t)((exp << 9U) | (period_us & 0x1FFU));

  /* Compute 4-bit checksum: inverted XOR nibbles */
  uint16_t c_val = value;
  uint8_t crc = (c_val ^ (c_val >> 4U) ^ (c_val >> 8U)) & 0x0FU;
  crc = (~crc) & 0x0FU;

  return (uint16_t)((value << 4U) | crc);
}
