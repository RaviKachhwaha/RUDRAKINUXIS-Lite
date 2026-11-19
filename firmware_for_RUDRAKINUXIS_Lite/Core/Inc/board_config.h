/**
 ******************************************************************************
 * @file           : board_config.h
 * @brief          : Pin assignments, peripheral parameters, electrical ratings,
 *                   and hardware definitions for RUDRAKINUXIS-Lite 6S 600W FOC
 * ESC. MCU: STM32G431CBU6 (UFQFPN48, 170 MHz) Gate Driver: TI DRV8300 Triple
 * Half-Bridge
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 RUDRAKINUXIS Project.
 * All rights reserved.
 *
 ******************************************************************************
 */

#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*============================================================================*/
/*                      ELECTRICAL & OPERATING LIMITS                         */
/*============================================================================*/

/** @brief Nominal battery pack cell count: 6S LiPo */
#define BATTERY_CELL_COUNT (6U)

/** @brief Maximum operating DC bus voltage (Over-Voltage Protection trip) */
#define VBUS_OVP_THRESHOLD_V (26.5f)

/** @brief Minimum operating DC bus voltage (Under-Voltage Lockout trip) */
#define VBUS_UVLO_THRESHOLD_V (18.0f)

/** @brief Battery nominal voltage (3.7V * 6) */
#define VBUS_NOMINAL_V (22.2f)

/** @brief Continuous electrical power rating (Watts) */
#define ESC_RATED_POWER_W (600.0f)

/** @brief Continuous RMS phase current rating (Amperes) */
#define CURRENT_RATED_CONT_A (27.0f)

/** @brief Maximum transient phase current rating (Amperes) */
#define CURRENT_MAX_PEAK_A (45.0f)

/** @brief Software Over-Current Protection (OCP) trip limit (Amperes) */
#define CURRENT_SW_OCP_LIMIT_A (55.0f)

/** @brief Software OCP persistence filter duration before trip (milliseconds)
 */
#define CURRENT_SW_OCP_PERSIST_MS (5U)

/** @brief Hardware Comparator OCP trip threshold (instantaneous gate shutdown)
 */
#define CURRENT_HW_OCP_LIMIT_A (65.0f)

/** @brief Thermal derating onset temperature (degrees Celsius) */
#define TEMP_DERATING_START_C (85.0f)

/** @brief Critical over-temperature shutdown trip (degrees Celsius) */
#define TEMP_CRITICAL_SHUTDOWN_C (105.0f)

/** @brief Rotor lock / stall timeout under commanded current (milliseconds) */
#define ROTOR_LOCK_TIMEOUT_MS (250U)

/** @brief Control input timeout watchdog (milliseconds) */
#define THROTTLE_SIGNAL_TIMEOUT_MS (100U)

/*============================================================================*/
/*                      TIMING & PWM CLOCK DEFINITIONS                        */
/*============================================================================*/

/** @brief Core CPU Frequency: 170 MHz */
#define SYSTEM_CORE_CLOCK_HZ (170000000UL)

/** @brief Inverter PWM carrier frequency: 24 kHz */
#define PWM_CARRIER_FREQ_HZ (24000UL)

/**
 * @brief Center-Aligned Mode 1 Auto-Reload Register (ARR) calculation:
 *        ARR = f_CPU / (2 * f_PWM)
 *        ARR = 170,000,000 / (2 * 24,000) = 3541.67 -> 3541
 *        Counting period = 2 * (3541 + 1) / 170,000,000 = 41.671 us (23.997
 * kHz)
 */
#define PWM_TIMER_ARR_VALUE (3541U)

/**
 * @brief Hardware Dead-Time in nanoseconds for DRV8300 shoot-through immunity.
 *        Gate driver propagation delay + MOSFET turn-off margin.
 */
#define PWM_DEADTIME_NS (350U)

/**
 * @brief Dead-Time Generator (DTG) calculation:
 *        T_DTS = 1 / 170 MHz = 5.8824 ns
 *        DTG = 350 ns / 5.8824 ns = 59.5 -> 60 ticks (0x3C)
 */
#define PWM_DEADTIME_TICKS (60U)

/** @brief FOC loop execution frequency (equals PWM carrier frequency) */
#define FOC_LOOP_FREQ_HZ (PWM_CARRIER_FREQ_HZ)

/** @brief FOC loop sampling period in seconds: 1 / 24000 s = 41.6667 us */
#define FOC_LOOP_PERIOD_SEC (1.0f / (float)FOC_LOOP_FREQ_HZ)

/*============================================================================*/
/*                      ANALOG FRONT-END & CURRENT SENSING                    */
/*============================================================================*/

/** @brief Low-side shunt resistor value in Ohms (0.5 milli-Ohms) */
#define SHUNT_RESISTOR_OHMS (0.0005f)

/** @brief Internal OPAMP Programmable Gain Amplifier (PGA) Gain setting (16x)
 */
#define OPAMP_PGA_GAIN_VALUE (16.0f)

/** @brief Microcontroller Analog-to-Digital Converter Reference Voltage */
#define ADC_VREF_VOLTAGE (3.3f)

/** @brief 12-Bit ADC full-scale resolution (2^12 - 1) */
#define ADC_FULL_SCALE_COUNTS (4095.0f)

/**
 * @brief Current sensor conversion scale factor:
 *        I_phase (A) = ADC_counts * (VREF / 4095) / (GAIN * R_SHUNT)
 *        Scale = 3.3 / (4095.0 * 16.0 * 0.0005) = 0.1007326 A/count
 */
#define CURRENT_SENSE_SCALE_FACTOR                                             \
  (ADC_VREF_VOLTAGE /                                                          \
   (ADC_FULL_SCALE_COUNTS * OPAMP_PGA_GAIN_VALUE * SHUNT_RESISTOR_OHMS))

/**
 * @brief DC Bus Voltage divider resistors:
 *        Top resistor (R1): 169 kOhms
 *        Bottom resistor (R2): 18 kOhms
 *        Divider ratio: (169 + 18) / 18 = 10.388889
 *        V_BUS = ADC_counts * (VREF / 4095) * 10.388889
 */
#define VBUS_DIVIDER_R1_OHMS (169000.0f)
#define VBUS_DIVIDER_R2_OHMS (18000.0f)
#define VBUS_DIVIDER_RATIO                                                     \
  ((VBUS_DIVIDER_R1_OHMS + VBUS_DIVIDER_R2_OHMS) / VBUS_DIVIDER_R2_OHMS)
#define VBUS_SENSE_SCALE_FACTOR                                                \
  ((ADC_VREF_VOLTAGE / ADC_FULL_SCALE_COUNTS) * VBUS_DIVIDER_RATIO)

/**
 * @brief BEMF phase voltage divider ratio:
 *        Top resistor: 33 kOhms, Bottom resistor: 3.3 kOhms -> Ratio = 11.0
 */
#define BEMF_DIVIDER_RATIO (11.0f)
#define BEMF_SENSE_SCALE_FACTOR                                                \
  ((ADC_VREF_VOLTAGE / ADC_FULL_SCALE_COUNTS) * BEMF_DIVIDER_RATIO)

/** @brief NTC Thermistor parameters for Steinhart-Hart / Beta equation */
#define NTC_NOMINAL_RESISTANCE_OHMS (10000.0f) /* 10 kOhms @ 25°C */
#define NTC_NOMINAL_TEMP_KELVIN (298.15f)      /* 25°C in Kelvin */
#define NTC_BETA_COEFFICIENT (3950.0f)         /* B25/85 Beta value */
#define NTC_PULLUP_RESISTOR_OHMS (10000.0f)    /* 10 kOhms pull-up to 3.3V */

/*============================================================================*/
/*                           PINOUT MAPPINGS                                  */
/*============================================================================*/

/*--- Inverter PWM: Advanced Control Timer (TIM1) ---*/
#define TIM1_PWM_TIMER TIM1

/* Phase U */
#define TIM1_CH1_PIN LL_GPIO_PIN_8
#define TIM1_CH1_PORT GPIOA
#define TIM1_CH1_AF LL_GPIO_AF_6

#define TIM1_CH1N_PIN LL_GPIO_PIN_13
#define TIM1_CH1N_PORT GPIOB
#define TIM1_CH1N_AF LL_GPIO_AF_6

/* Phase V */
#define TIM1_CH2_PIN LL_GPIO_PIN_9
#define TIM1_CH2_PORT GPIOA
#define TIM1_CH2_AF LL_GPIO_AF_6

#define TIM1_CH2N_PIN LL_GPIO_PIN_14
#define TIM1_CH2N_PORT GPIOB
#define TIM1_CH2N_AF LL_GPIO_AF_6

/* Phase W */
#define TIM1_CH3_PIN LL_GPIO_PIN_10
#define TIM1_CH3_PORT GPIOA
#define TIM1_CH3_AF LL_GPIO_AF_6

#define TIM1_CH3N_PIN LL_GPIO_PIN_15
#define TIM1_CH3N_PORT GPIOB
#define TIM1_CH3N_AF LL_GPIO_AF_4

/* Emergency Break Input (BKIN) */
#define TIM1_BKIN_PIN LL_GPIO_PIN_12
#define TIM1_BKIN_PORT GPIOB
#define TIM1_BKIN_AF LL_GPIO_AF_6

/*--- Gate Driver DRV8300 Enable Pin ---*/
#define DRV_ENABLE_PIN LL_GPIO_PIN_13
#define DRV_ENABLE_PORT GPIOC

/*--- Current Sensing Analog Inputs & Internal OPAMPs ---*/
/* OPAMP1 -> Phase U Current */
#define OPAMP1_INP_PIN LL_GPIO_PIN_1
#define OPAMP1_INP_PORT GPIOA

/* OPAMP2 -> Phase V Current */
#define OPAMP2_INP_PIN LL_GPIO_PIN_7
#define OPAMP2_INP_PORT GPIOA

/* OPAMP3 -> Phase W Current */
#define OPAMP3_INP_PIN LL_GPIO_PIN_0
#define OPAMP3_INP_PORT GPIOB

/*--- Voltage & Temperature Sensing (ADC Regular Channels) ---*/
/* DC Bus Voltage */
#define VBUS_ADC_PIN LL_GPIO_PIN_0
#define VBUS_ADC_PORT GPIOA
#define VBUS_ADC_CHANNEL LL_ADC_CHANNEL_1

/* BEMF Phase U */
#define BEMF_U_ADC_PIN LL_GPIO_PIN_2
#define BEMF_U_ADC_PORT GPIOB
#define BEMF_U_ADC_CHANNEL LL_ADC_CHANNEL_12

/* BEMF Phase V */
#define BEMF_V_ADC_PIN LL_GPIO_PIN_11
#define BEMF_V_ADC_PORT GPIOB
#define BEMF_V_ADC_CHANNEL LL_ADC_CHANNEL_14

/* BEMF Phase W */
#define BEMF_W_ADC_PIN LL_GPIO_PIN_4
#define BEMF_W_ADC_PORT GPIOC
#define BEMF_W_ADC_CHANNEL LL_ADC_CHANNEL_5

/* On-board NTC Thermistor */
#define NTC_ADC_PIN LL_GPIO_PIN_4
#define NTC_ADC_PORT GPIOA
#define NTC_ADC_CHANNEL LL_ADC_CHANNEL_17

/*--- Telemetry & CLI: USART1 (Silicon Labs CP2102N Bridge) ---*/
#define CLI_USART USART1
#define CLI_USART_BAUDRATE (115200UL)
#define CLI_USART_TX_PIN LL_GPIO_PIN_6
#define CLI_USART_TX_PORT GPIOB
#define CLI_USART_TX_AF LL_GPIO_AF_7

#define CLI_USART_RX_PIN LL_GPIO_PIN_7
#define CLI_USART_RX_PORT GPIOB
#define CLI_USART_RX_AF LL_GPIO_AF_7

/*--- CAN FD Interface (NXP TJA1051T/3 Transceiver) ---*/
#define CANFD_INTERFACE FDCAN1
#define CANFD_RX_PIN LL_GPIO_PIN_8
#define CANFD_RX_PORT GPIOB
#define CANFD_RX_AF LL_GPIO_AF_9

#define CANFD_TX_PIN LL_GPIO_PIN_9
#define CANFD_TX_PORT GPIOB
#define CANFD_TX_AF LL_GPIO_AF_9

/*--- Flight Controller Throttle Input (DShot300/600 & PWM) ---*/
#define DSHOT_TIM TIM2
#define DSHOT_TIM_CHANNEL LL_TIM_CHANNEL_CH1
#define DSHOT_INPUT_PIN LL_GPIO_PIN_15
#define DSHOT_INPUT_PORT GPIOA
#define DSHOT_INPUT_AF LL_GPIO_AF_1

/*--- Status LEDs ---*/
#define LED1_GREEN_PIN LL_GPIO_PIN_4
#define LED1_GREEN_PORT GPIOB

#define LED2_BLUE_PIN LL_GPIO_PIN_5
#define LED2_BLUE_PORT GPIOB

/* LED Control Helper Macros */
#define LED1_GREEN_ON() LL_GPIO_SetOutputPin(LED1_GREEN_PORT, LED1_GREEN_PIN)
#define LED1_GREEN_OFF() LL_GPIO_ResetOutputPin(LED1_GREEN_PORT, LED1_GREEN_PIN)
#define LED1_GREEN_TOGGLE() LL_GPIO_TogglePin(LED1_GREEN_PORT, LED1_GREEN_PIN)

#define LED2_BLUE_ON() LL_GPIO_SetOutputPin(LED2_BLUE_PORT, LED2_BLUE_PIN)
#define LED2_BLUE_OFF() LL_GPIO_ResetOutputPin(LED2_BLUE_PORT, LED2_BLUE_PIN)
#define LED2_BLUE_TOGGLE() LL_GPIO_TogglePin(LED2_BLUE_PORT, LED2_BLUE_PIN)

#define DRV_ENABLE_HIGH() LL_GPIO_SetOutputPin(DRV_ENABLE_PORT, DRV_ENABLE_PIN)
#define DRV_ENABLE_LOW() LL_GPIO_ResetOutputPin(DRV_ENABLE_PORT, DRV_ENABLE_PIN)

/*--- 5-Pin SWD Programming & Debug Interface (ST-Link V2 / V3) ---*/
/**
 * @brief  Hardware 5-Pin Debug Header Configuration:
 *         Pin 1: +3V3 (Target Reference Voltage from onboard 3.3V LDO)
 *         Pin 2: SWDIO (Serial Wire Data Input/Output) -> PA13 (AF0)
 *         Pin 3: SWCLK (Serial Wire Clock)            -> PA14 (AF0)
 *         Pin 4: GND   (Common Digital Ground Reference)
 *         Pin 5: NRST  (Hardware System Reset Line   -> Pin 7 / NRST)
 *
 * @note   Having the 5th pin (NRST) enables hardware "Connect Under Reset"
 *         recovery, protecting the ESC against bricking if code halts or
 *         enters an unexpected state, and allows automatic reset post-flash.
 */
#define SWD_SWDIO_PIN        LL_GPIO_PIN_13
#define SWD_SWDIO_PORT       GPIOA
#define SWD_SWDIO_AF         LL_GPIO_AF_0

#define SWD_SWCLK_PIN        LL_GPIO_PIN_14
#define SWD_SWCLK_PORT       GPIOA
#define SWD_SWCLK_AF         LL_GPIO_AF_0

#define SWD_NRST_PIN_NUMBER  (7U)  /* STM32G431CBU6 UFQFPN48 Pin 7 */

/*============================================================================*/
/*                      DMA BUFFER ALLOCATION SIZES                           */
/*============================================================================*/

/** @brief Regular ADC channels conversion buffer size (VBUS, BEMF_U, V, W, NTC,
 * VREFINT) */
#define ADC_REGULAR_BUFFER_SIZE (6U)

/** @brief Index definitions for regular ADC buffer */
#define ADC_IDX_VBUS (0U)
#define ADC_IDX_BEMF_U (1U)
#define ADC_IDX_BEMF_V (2U)
#define ADC_IDX_BEMF_W (3U)
#define ADC_IDX_NTC (4U)
#define ADC_IDX_VREFINT (5U)

/* Global DMA Buffer for Regular ADC Channels */
extern volatile uint16_t g_adc_regular_buffer[ADC_REGULAR_BUFFER_SIZE];

/*============================================================================*/
/*                      FUNCTION PROTOTYPES                                   */
/*============================================================================*/

/**
 * @brief  Configures the system clock to 170 MHz using internal 16MHz HSI &
 * PLL.
 * @retval None
 */
void Board_SystemClock_Config(void);

/**
 * @brief  Initializes all GPIO pins according to board specification.
 * @retval None
 */
void Board_GPIO_Init(void);

/**
 * @brief  Initializes the high-speed complementary PWM timer (TIM1) for
 * DRV8300.
 * @retval None
 */
void Board_TIM1_PWM_Init(void);

/**
 * @brief  Initializes internal OPAMPs (OPAMP1, 2, 3) in PGA mode (16x).
 * @retval None
 */
void Board_OPAMP_Init(void);

/**
 * @brief  Initializes internal over-current comparator (COMP1) tied to
 * TIM1_BKIN.
 * @retval None
 */
void Board_COMP_OCP_Init(void);

/**
 * @brief  Initializes ADC1 and ADC2 in dual simultaneous injected mode for FOC.
 * @retval None
 */
void Board_ADC_Dual_Init(void);

/**
 * @brief  Initializes hardware CORDIC accelerator coprocessor.
 * @retval None
 */
void Board_CORDIC_Init(void);

/**
 * @brief  Initializes USART1 with DMA for non-blocking telemetry CLI.
 * @retval None
 */
void Board_USART1_Init(void);

/**
 * @brief  Initializes FDCAN1 interface for telemetry and actuator commands.
 * @retval None
 */
void Board_FDCAN1_Init(void);

/**
 * @brief  Initializes TIM2 and DMA for DShot300/600 packet capture.
 * @retval None
 */
void Board_DShot_Timer_Init(void);

/**
 * @brief  Master initialization calling all peripheral setup routines in order.
 * @retval None
 */
void Board_Init_All(void);

/**
 * @brief  Hardware emergency stop: clears Main Output Enable (MOE) on TIM1.
 * @retval None
 */
static inline void Board_EmergencyStop_Hardware(void) {
  /* Clear MOE (Main Output Enable) bit in TIM1 BDTR to immediately disable all
   * gate outputs */
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  DRV_ENABLE_LOW();
}

/**
 * @brief  Hardware gate output enable: sets Main Output Enable (MOE) on TIM1.
 * @retval None
 */
static inline void Board_EnableGates_Hardware(void) {
  DRV_ENABLE_HIGH();
  /* Set MOE bit in TIM1 BDTR */
  TIM1->BDTR |= TIM_BDTR_MOE;
}

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_CONFIG_H */
