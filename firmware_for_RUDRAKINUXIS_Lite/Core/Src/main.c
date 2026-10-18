/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Production-grade firmware for RUDRAKINUXIS-Lite 6S 600W FOC
 * ESC. Target: STM32G431CBU6 (Arm Cortex-M4F @ 170MHz) Hardware: TI DRV8300,
 * 0.5mOhm shunts, NTC, CP2102N, TJA1051T/3.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 RUDRAKINUXIS Project.
 * All rights reserved.
 *
 ******************************************************************************
 */

#include "main.h"
#include "board_config.h"
#include "bootloader_jump.h"
#include "canfd_interface.h"
#include "dshot_decoder.h"
#include "foc_core.h"
#include "foc_math.h"
#include "observer_smo.h"
#include "safety_supervisor.h"
#include "telemetry_cli.h"

/* Private timing variables */
static uint32_t s_last_can_tx_tick = 0U;

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /*
   * 1. Early Bootloader Check:
   *    MUST be called before any clocks, caching, or peripherals are
   * initialized. If the persistent backup register contains 0xDEADBEEF, clears
   * the signature, remaps system flash, configures MSP, and jumps directly to
   * the ST Factory ROM Bootloader at 0x1FFF0000 for single-click USB-C
   * flashing.
   */
  Bootloader_CheckAndJump();

  /* 2. Initialize HAL Library */
  HAL_Init();

  /*
   * 3. Comprehensive Hardware Board Initialization:
   *    - 170 MHz PLL clock from internal 16MHz HSI (Range 1 Boost Mode)
   *    - DWT cycle counter for microsecond timing & profiling
   *    - GPIO mappings for PWM, analog, communications, LEDs
   *    - Hardware CORDIC coprocessor
   *    - Internal OPAMPs in PGA mode (16x)
   *    - COMP1 over-current hardware trip (< 50 ns to TIM1 BKIN)
   *    - Dual ADC1 & ADC2 in Injected Simultaneous Mode
   *    - TIM1 Complementary PWM with 350 ns dead-time & TRGO2 update event
   *    - USART1 & FDCAN1 interfaces
   */
  Board_Init_All();

  /* 4. Initialize Core Modules */
  Safety_Supervisor_Init();
  FOC_Core_Init(&g_foc);
  DShot_Decoder_Init();
  CANFD_Init();
  Telemetry_CLI_Init();

  /* Welcome Banner over Serial CLI */
  Telemetry_CLI_Printf("\r\n========================================\r\n"
                       "  RUDRAKINUXIS-Lite 6S 600W FOC ESC     \r\n"
                       "  STM32G431CBU6 @ 170MHz | 24kHz SVPWM  \r\n"
                       "  Firmware v1.0.0 (Production Build)    \r\n"
                       "========================================\r\n");

  /* Main Super-Loop */
  while (1) {
    /*
     * A. High-Speed Flight Controller Throttle Input (DShot300/600/PWM):
     *    Processes DMA pulse captures and decodes 16-bit DShot frames.
     */
    if (DShot_Decoder_Process()) {
      if (g_dshot.normalized_throttle > 0.01f) {
        /* Automatically arm if valid positive throttle is received */
        if (!g_foc.is_armed && (g_foc.state == FOC_STATE_DISARMED)) {
          FOC_Core_Arm(&g_foc);
        }

        /* Map throttle to current setpoint Iq */
        float target_iq = g_dshot.normalized_throttle * g_foc.max_iq_allowed_a;
        FOC_Core_SetTargetCurrent(&g_foc, target_iq);
      } else {
        /* Zero throttle: set torque current to zero */
        FOC_Core_SetTargetCurrent(&g_foc, 0.0f);
      }
    }

    /*
     * B. CAN FD / DroneCAN Actuator Command Reception:
     *    Processes throttle frames from external flight computer.
     */
    if (CANFD_Process_Rx()) {
      if (g_can.arm_commanded && !g_foc.is_armed &&
          (g_foc.state == FOC_STATE_DISARMED)) {
        FOC_Core_Arm(&g_foc);
      } else if (!g_can.arm_commanded && g_foc.is_armed) {
        FOC_Core_Disarm(&g_foc);
      }

      if (g_foc.is_armed) {
        float target_iq = g_can.target_throttle * g_foc.max_iq_allowed_a;
        FOC_Core_SetTargetCurrent(&g_foc, target_iq);
      }
    }

    /*
     * C. Periodic CAN FD Telemetry Broadcast (50 Hz / 20 ms):
     *    Streams voltage, current, mechanical RPM, board temperature, and
     * status.
     */
    uint32_t now = HAL_GetTick();
    if ((now - s_last_can_tx_tick) >= 20U) {
      s_last_can_tx_tick = now;

      uint8_t status_flags = 0U;
      if (g_foc.is_armed)
        status_flags |= CAN_STATUS_ARMED_BIT;
      if (g_foc.state == FOC_STATE_CLOSED_LOOP)
        status_flags |= CAN_STATUS_CLOSED_LOOP_BIT;
      if (g_safety.active_faults != 0U)
        status_flags |= CAN_STATUS_FAULT_BIT;
      if (g_safety.active_faults & FAULT_FLAG_SW_OCP)
        status_flags |= CAN_STATUS_OCP_BIT;
      if (g_safety.active_faults & FAULT_FLAG_OVP)
        status_flags |= CAN_STATUS_OVP_BIT;
      if (g_safety.active_faults & FAULT_FLAG_UVLO)
        status_flags |= CAN_STATUS_UVLO_BIT;
      if (g_safety.active_faults & FAULT_FLAG_OTP)
        status_flags |= CAN_STATUS_OTP_BIT;
      if (g_safety.active_faults & FAULT_FLAG_ROTOR_LOCK)
        status_flags |= CAN_STATUS_STALL_BIT;

      CANFD_TransmitTelemetry(g_foc.v_bus, g_foc.i_dq.q, g_foc.smo.rpm_mech,
                              g_safety.measured_temp_c, status_flags);
    }

    /*
     * D. Serial Telemetry Streaming & Interactive CLI:
     *    Streams SimpleFOC Studio CSV strings and parses ASCII commands.
     */
    Telemetry_CLI_Stream_Step();
    Telemetry_CLI_Process();
  }
}

/**
 * @brief  SysTick Handler: Increment HAL tick and run 1 kHz Safety Supervisor.
 */
void SysTick_Handler(void) {
  HAL_IncTick();

  /* Execute deterministic 1 kHz safety task */
  Safety_Supervisor_Task_1kHz();
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* Emergency stop */
  Board_EmergencyStop_Hardware();
  __disable_irq();

  while (1) {
    /* Strobe red/blue error LED */
    LED2_BLUE_TOGGLE();
    for (volatile uint32_t i = 0; i < 500000; i++) {
    }
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { Error_Handler(); }
#endif
