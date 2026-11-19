/**
 ******************************************************************************
 * @file           : board_config.c
 * @brief          : Complete hardware peripheral configuration, clock tree,
 *                   DMA, high-speed PWM timer, dual ADC, OPAMP, and comparator
 * setup. RUDRAKINUXIS-Lite 6S 600W FOC ESC.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 RUDRAKINUXIS Project.
 * All rights reserved.
 *
 ******************************************************************************
 */

#include "board_config.h"

/* Global DMA Buffer for Regular ADC Channels */
volatile uint16_t g_adc_regular_buffer[ADC_REGULAR_BUFFER_SIZE] = {0};

/* External handles / declarations */
extern void FOC_ADC_Injected_Callback(void);

/**
 * @brief  System Clock Configuration: 170 MHz using HSI (16 MHz) + PLL.
 *         Configures Flash latency to 4WS, enables Range 1 Boost mode,
 *         and switches SYSCLK to PLLR output at 170 MHz.
 * @retval None
 */
void Board_SystemClock_Config(void) {
  /* Enable Power Controller Clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /*
   * In STM32G4, to operate at frequencies above 150 MHz up to 170 MHz,
   * the system must be placed in Voltage Scaling Range 1 Boost Mode.
   * Clear R1MODE bit in PWR_CR5 to activate Boost Mode.
   */
  LL_PWR_EnableRange1BoostMode();

  /* Configure Flash Pre-fetch buffer and 4 Wait States (required for 170 MHz
   * at 3.3V) */
  LL_FLASH_EnablePrefetch();
  LL_FLASH_EnableInstCache();
  LL_FLASH_EnableDataCache();
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);

  /* Verify Flash Latency was applied */
  while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4) {
  }

  /* Enable High Speed Internal (HSI) 16 MHz oscillator */
  LL_RCC_HSI_Enable();
  while (LL_RCC_HSI_IsReady() != 1U) {
    /* Wait for HSI ready */
  }

  /* Set HSI calibration trimming to default */
  LL_RCC_HSI_SetCalibTrimming(64U);

  /*
   * Configure Main PLL:
   * PLL Source = HSI (16 MHz)
   * PLLM = /4  -> 4 MHz VCO input (must be between 2.66 and 8 MHz)
   * PLLN = *85 -> 340 MHz VCO output (must be between 64 and 344 MHz)
   * PLLR = /2  -> 170 MHz System Clock (PLLCLK)
   * PLLP = /2  -> 170 MHz (Optional ADC/Audio clock)
   * PLLQ = /2  -> 170 MHz (Optional FDCAN/USB clock)
   */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_4, 85U,
                              LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();

  while (LL_RCC_PLL_IsReady() != 1U) {
    /* Wait for PLL lock */
  }

  /* Set AHB prescaler = 1 (HCLK = 170 MHz) */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Set APB1 prescaler = 1 (PCLK1 = 170 MHz) */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  /* Set APB2 prescaler = 1 (PCLK2 = 170 MHz) */
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Switch System Clock source to PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    /* Wait for system clock switch */
  }

  /* Update SystemCoreClock variable */
  SystemCoreClock = SYSTEM_CORE_CLOCK_HZ;

  /* Configure SysTick to 1 ms periodic interrupt */
  LL_Init1msTick(SYSTEM_CORE_CLOCK_HZ);
  LL_SetSystemCoreClock(SYSTEM_CORE_CLOCK_HZ);
}

/**
 * @brief  Initializes GPIO pins for PWM outputs, analog inputs, communications,
 *         gate driver enable, and status LEDs.
 * @retval None
 */
void Board_GPIO_Init(void) {
  /* Enable GPIO Bus Clocks */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA |
                           LL_AHB2_GRP1_PERIPH_GPIOB |
                           LL_AHB2_GRP1_PERIPH_GPIOC);

  LL_GPIO_InitTypeDef gpio_init = {0};

  /*--- 1. Status LEDs (PB4 = Green, PB5 = Blue) ---*/
  LL_GPIO_ResetOutputPin(LED1_GREEN_PORT, LED1_GREEN_PIN | LED2_BLUE_PIN);
  gpio_init.Pin = LED1_GREEN_PIN | LED2_BLUE_PIN;
  gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_LOW;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &gpio_init);

  /*--- 2. Gate Driver DRV8300 Enable Pin (PC13) ---*/
  DRV_ENABLE_LOW(); /* Start in disabled state */
  gpio_init.Pin = DRV_ENABLE_PIN;
  gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(DRV_ENABLE_PORT, &gpio_init);

  /*--- 3. TIM1 Complementary PWM High-Side (PA8, PA9, PA10 - AF6) ---*/
  gpio_init.Pin = TIM1_CH1_PIN | TIM1_CH2_PIN | TIM1_CH3_PIN;
  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_DOWN;
  gpio_init.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOA, &gpio_init);

  /*--- 4. TIM1 Complementary PWM Low-Side (PB13, PB14 - AF6; PB15 - AF4) ---*/
  gpio_init.Pin = TIM1_CH1N_PIN | TIM1_CH2N_PIN;
  gpio_init.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOB, &gpio_init);

  gpio_init.Pin = TIM1_CH3N_PIN;
  gpio_init.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &gpio_init);

  /*--- 5. TIM1 Hardware Break Input (PB12 - AF6) ---*/
  gpio_init.Pin = TIM1_BKIN_PIN;
  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_UP;
  gpio_init.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(TIM1_BKIN_PORT, &gpio_init);

  /*--- 6. Analog Current Sensing Inputs for Internal OPAMPs ---*/
  /* OPAMP1_INP (PA1), OPAMP2_INP (PA7) */
  gpio_init.Pin = OPAMP1_INP_PIN | OPAMP2_INP_PIN;
  gpio_init.Mode = LL_GPIO_MODE_ANALOG;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &gpio_init);

  /* OPAMP3_INP (PB0) */
  gpio_init.Pin = OPAMP3_INP_PIN;
  LL_GPIO_Init(GPIOB, &gpio_init);

  /*--- 7. Analog Regular Sensing Channels (VBUS, BEMF_U/V/W, NTC) ---*/
  /* VBUS (PA0), NTC (PA4) */
  gpio_init.Pin = VBUS_ADC_PIN | NTC_ADC_PIN;
  gpio_init.Mode = LL_GPIO_MODE_ANALOG;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &gpio_init);

  /* BEMF_U (PB2), BEMF_V (PB11) */
  gpio_init.Pin = BEMF_U_ADC_PIN | BEMF_V_ADC_PIN;
  LL_GPIO_Init(GPIOB, &gpio_init);

  /* BEMF_W (PC4) */
  gpio_init.Pin = BEMF_W_ADC_PIN;
  LL_GPIO_Init(GPIOC, &gpio_init);

  /*--- 8. Telemetry USART1 (PB6 TX, PB7 RX - AF7) ---*/
  gpio_init.Pin = CLI_USART_TX_PIN | CLI_USART_RX_PIN;
  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_UP;
  gpio_init.Alternate = CLI_USART_TX_AF;
  LL_GPIO_Init(GPIOB, &gpio_init);

  /*--- 9. CAN FD Transceiver (PB8 RX, PB9 TX - AF9) ---*/
  gpio_init.Pin = CANFD_RX_PIN | CANFD_TX_PIN;
  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_UP;
  gpio_init.Alternate = CANFD_RX_AF;
  LL_GPIO_Init(GPIOB, &gpio_init);

  /*--- 10. Flight Controller DShot / PWM Input (PA15 - AF1, TIM2_CH1) ---*/
  gpio_init.Pin = DSHOT_INPUT_PIN;
  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_DOWN;
  gpio_init.Alternate = DSHOT_INPUT_AF;
  LL_GPIO_Init(GPIOA, &gpio_init);

  /*--- 11. 5-Pin SWD Programming & Debug Interface (ST-Link V2 / V3) ---*/
  /* SWDIO (PA13) and SWCLK (PA14) are preserved in their default AF0 (SWD) state.
   * Pin 7 is hardware NRST (System Reset) with internal pull-up filter.
   * Both PA13 and PA14 are strictly protected and never repurposed as standard GPIOs,
   * guaranteeing reliable SWD flashing and full "Connect Under Reset" recovery. */
}

/**
 * @brief  Initializes High-Speed Inverter Timer (TIM1) with complementary PWM,
 *         350 ns dead-time, Center-Aligned Mode 1, and TRGO2 for ADC
 * triggering.
 * @retval None
 */
void Board_TIM1_PWM_Init(void) {
  /* Enable TIM1 Peripheral Clock */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* Disable counter before configuration */
  LL_TIM_DisableCounter(TIM1);

  /*
   * Configure TIM1 Time Base:
   * System Clock = 170 MHz
   * Prescaler = 0
   * Mode: Center-Aligned Mode 1 (Counts up and down, output compare interrupt
   * set when counting down) Auto-Reload = PWM_TIMER_ARR_VALUE (3541) PWM
   * frequency = 170,000,000 / (2 * 3541.67) = 24.0 kHz
   */
  LL_TIM_InitTypeDef tim_base = {0};
  tim_base.Prescaler = 0U;
  tim_base.CounterMode = LL_TIM_COUNTERMODE_CENTER_UP_DOWN;
  tim_base.Autoreload = PWM_TIMER_ARR_VALUE;
  tim_base.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  tim_base.RepetitionCounter =
      1U; /* Update event every full PWM period (valley) */
  LL_TIM_Init(TIM1, &tim_base);

  /* Enable ARR Preload */
  LL_TIM_EnableARRPreload(TIM1);

  /*
   * Configure PWM Channels (CH1, CH2, CH3) in PWM Mode 1:
   * Channels are active while Counter < CCR, inactive when Counter > CCR.
   * Preload enabled on all channels to update simultaneously on valley event.
   */
  LL_TIM_OC_InitTypeDef oc_init = {0};
  oc_init.OCMode = LL_TIM_OCMODE_PWM1;
  oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
  oc_init.OCNState = LL_TIM_OCSTATE_ENABLE;
  oc_init.CompareValue = 0U; /* Start with 0% duty */
  oc_init.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  oc_init.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  oc_init.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  oc_init.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;

  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &oc_init);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);

  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &oc_init);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);

  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &oc_init);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);

  /*
   * Configure Break and Dead-Time Register (BDTR):
   * Hardware Dead-Time = 350 ns -> 60 ticks (PWM_DEADTIME_TICKS).
   * Break 1 Enabled, Active High, Auto Output Enable (AOE) disabled for safety.
   */
  LL_TIM_BDTR_InitTypeDef bdtr_init = {0};
  bdtr_init.OSSRState = LL_TIM_OSSR_ENABLE;
  bdtr_init.OSSIState = LL_TIM_OSSI_ENABLE;
  bdtr_init.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  bdtr_init.DeadTime = PWM_DEADTIME_TICKS;
  bdtr_init.BreakState = LL_TIM_BREAK_ENABLE;
  bdtr_init.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  bdtr_init.BreakFilter =
      LL_TIM_BREAK_FILTER_FDIV2_N6; /* Digital glitch filter */
  bdtr_init.Break2State = LL_TIM_BREAK2_DISABLE;
  bdtr_init.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &bdtr_init);

  /*
   * Configure Trigger Output 2 (TRGO2) on Update Event (Valley):
   * This occurs when the counter reaches 0 (all high-sides OFF, low-sides
   * conducting). This triggers synchronized ADC Injected conversion at the
   * exact center of low-side on-time!
   */
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_UPDATE);

  /* Route external BKIN pin and internal Comparator COMP1 output to TIM1 BKIN for instant hardware cutoff */
  LL_TIM_EnableBreakInputSource(TIM1, LL_TIM_BREAK_INPUT_BKIN, LL_TIM_BKIN_SOURCE_BKIN);
  LL_TIM_EnableBreakInputSource(TIM1, LL_TIM_BREAK_INPUT_BKIN, LL_TIM_BKIN_SOURCE_BKCOMP1);

  /* Generate an update event to force register transfer */
  LL_TIM_GenerateEvent_UPDATE(TIM1);

  /* Clear all interrupt flags */
  LL_TIM_ClearFlag_UPDATE(TIM1);
  LL_TIM_ClearFlag_BRK(TIM1);
}

/**
 * @brief  Initializes Internal OPAMPs (OPAMP1, 2, 3) in Programmable Gain
 *         Amplifier (PGA) mode (16x) with non-inverting inputs tied to shunts.
 * @retval None
 */
void Board_OPAMP_Init(void) {
  /* Enable SYSCFG and OPAMP clock */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

  /*--- OPAMP1 (Phase U Current) ---*/
  /* Non-inverting input: VINP0 (PA1) */
  LL_OPAMP_SetInputNonInverting(OPAMP1, LL_OPAMP_INPUT_NONINVERT_IO0);
  /* Inverting input: Internal PGA connected to internal resistor ladder */
  LL_OPAMP_SetInputInverting(OPAMP1, LL_OPAMP_INPUT_INVERT_CONNECT_NO);
  /* Gain: 16x */
  LL_OPAMP_SetPGAGain(OPAMP1, LL_OPAMP_PGA_GAIN_16_OR_MINUS_15);
  /* Internal connection to ADC1 Channel 3 */
  LL_OPAMP_Enable(OPAMP1);

  /*--- OPAMP2 (Phase V Current) ---*/
  /* Non-inverting input: VINP0 (PA7) */
  LL_OPAMP_SetInputNonInverting(OPAMP2, LL_OPAMP_INPUT_NONINVERT_IO0);
  LL_OPAMP_SetInputInverting(OPAMP2, LL_OPAMP_INPUT_INVERT_CONNECT_NO);
  LL_OPAMP_SetPGAGain(OPAMP2, LL_OPAMP_PGA_GAIN_16_OR_MINUS_15);
  /* Internal connection to ADC2 Channel 3 */
  LL_OPAMP_Enable(OPAMP2);

  /*--- OPAMP3 (Phase W Current) ---*/
  /* Non-inverting input: VINP0 (PB0) */
  LL_OPAMP_SetInputNonInverting(OPAMP3, LL_OPAMP_INPUT_NONINVERT_IO0);
  LL_OPAMP_SetInputInverting(OPAMP3, LL_OPAMP_INPUT_INVERT_CONNECT_NO);
  LL_OPAMP_SetPGAGain(OPAMP3, LL_OPAMP_PGA_GAIN_16_OR_MINUS_15);
  /* Internal connection to ADC2 Channel 1 */
  LL_OPAMP_Enable(OPAMP3);
}

/**
 * @brief  Initializes internal analog comparator (COMP1) for instantaneous
 *         hardware over-current protection (<50 ns latency).
 * @retval None
 */
void Board_COMP_OCP_Init(void) {
  /* Enable COMP clock (via SYSCFG) */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

  /*
   * COMP1 Configuration:
   * Non-inverting input: OPAMP1 output (Phase U current amplifier).
   * Inverting input: 3/4 VREFINT (~0.91V) or 1/2 VREFINT (~0.61V).
   * 0.61V with 16x gain and 0.5mOhm corresponds to:
   * I = 0.61 / (16 * 0.0005) = 76.25 A peak.
   * Output: Active High, routed internally to TIM1 BKIN.
   */
  LL_COMP_InitTypeDef comp_init = {0};
  comp_init.InputPlus = LL_COMP_INPUT_PLUS_IO1; /* OPAMP1 internal output */
  comp_init.InputMinus = LL_COMP_INPUT_MINUS_3_4VREFINT;
  comp_init.InputHysteresis = LL_COMP_HYSTERESIS_MEDIUM;
  comp_init.OutputPolarity = LL_COMP_OUTPUTPOL_NONINVERTED;
  comp_init.OutputBlankingSource = LL_COMP_BLANKINGSRC_NONE;
  LL_COMP_Init(COMP1, &comp_init);

  /* Enable COMP1 */
  LL_COMP_Enable(COMP1);
}

/**
 * @brief  Initializes Dual ADC1 and ADC2 in Injected Simultaneous Mode.
 *         ADC1 Injected: OPAMP1 output (Phase U)
 *         ADC2 Injected: OPAMP2 output (Phase V)
 *         Trigger: TIM1 TRGO2 (Valley event, center of low-side on-time).
 *         Regular group: DMA round-robin conversion of VBUS, BEMF, NTC,
 * VREFINT.
 * @retval None
 */
void Board_ADC_Dual_Init(void) {
  /* Enable ADC1 & ADC2 Clocks */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

  /* Enable DMA1 for regular channels */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);

  /* Set ADC Clock to Synchronous Clock / 1 (HCLK / 1 = 170 MHz / 2 or / 4 via
   * prescaler) */
  LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1),
                        LL_ADC_CLOCK_SYNC_PCLK_DIV4);

  /* Enable internal voltage reference channel */
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
                                 LL_ADC_PATH_INTERNAL_VREFINT);

  /* Exit deep power-down and enable voltage regulators for ADC1 and ADC2 */
  LL_ADC_DisableDeepPowerDown(ADC1);
  LL_ADC_EnableInternalRegulator(ADC1);

  LL_ADC_DisableDeepPowerDown(ADC2);
  LL_ADC_EnableInternalRegulator(ADC2);

  /* Wait for ADC voltage regulators startup time (20 us) */
  DWT_Init();
  uint32_t start_cycles = DWT_GetCycles();
  while ((DWT_GetCycles() - start_cycles) < (20U * 170U)) {
  }

  /* Calibrate ADC1 */
  LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
  while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0U) {
  }

  /* Calibrate ADC2 */
  LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
  while (LL_ADC_IsCalibrationOnGoing(ADC2) != 0U) {
  }

  /* Enable ADC1 */
  LL_ADC_Enable(ADC1);
  while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0U) {
  }

  /* Enable ADC2 */
  LL_ADC_Enable(ADC2);
  while (LL_ADC_IsActiveFlag_ADRDY(ADC2) == 0U) {
  }

  /* Configure Dual ADC Common Mode: Injected Simultaneous Mode */
  LL_ADC_SetMultimode(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_MULTI_DUAL_INJ_SIMULT);

  /*
   * Configure ADC1 Injected Channel:
   * Injected Rank 1: Channel 3 (OPAMP1 internal output, Phase U current)
   * Sampling time: 6.5 ADC cycles
   * Trigger: TIM1 TRGO2, Rising Edge
   */
  LL_ADC_INJ_SetSequencerLength(ADC1, LL_ADC_INJ_SEQ_SCAN_DISABLE);
  LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_3);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3,
                                LL_ADC_SAMPLINGTIME_6CYCLES_5);
  LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2);
  LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);

  /*
   * Configure ADC2 Injected Channel:
   * Injected Rank 1: Channel 3 (OPAMP2 internal output, Phase V current)
   * Sampling time: 6.5 ADC cycles
   * Triggered in synchronization with Master ADC1
   */
  LL_ADC_INJ_SetSequencerLength(ADC2, LL_ADC_INJ_SEQ_SCAN_DISABLE);
  LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_3);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_3,
                                LL_ADC_SAMPLINGTIME_6CYCLES_5);
  LL_ADC_INJ_SetTriggerSource(ADC2, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2);
  LL_ADC_INJ_SetTriggerEdge(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);

  /*
   * Configure ADC1 Regular Sequencer for DMA round-robin:
   * Rank 1: Channel 1 (PA0, VBUS)
   * Rank 2: Channel 12 (PB2, BEMF_U)
   * Rank 3: Channel 14 (PB11, BEMF_V)
   * Rank 4: Channel 5 (PC4, BEMF_W)
   * Rank 5: Channel 17 (PA4, NTC Thermistor)
   * Rank 6: Channel 18 (VREFINT internal)
   */
  LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_6RANKS);
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_12);
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_14);
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_5);
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_5, LL_ADC_CHANNEL_17);
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_6, LL_ADC_CHANNEL_VREFINT);

  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1,
                                LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_12,
                                LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_14,
                                LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5,
                                LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_17,
                                LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT,
                                LL_ADC_SAMPLINGTIME_47CYCLES_5);

  /* Configure ADC1 Regular Continuous mode with DMA */
  LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_CONTINUOUS);
  LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

  /* Configure DMA1 Channel 1 for ADC1 Regular conversions */
  LL_DMA_ConfigTransfer(
      DMA1, LL_DMA_CHANNEL_1,
      LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_MODE_CIRCULAR |
          LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
          LL_DMA_PDATAALIGN_HALFWORD | LL_DMA_MDATAALIGN_HALFWORD |
          LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)(uintptr_t)&(ADC1->DR));
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1,
                          (uint32_t)(uintptr_t)g_adc_regular_buffer);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADC_REGULAR_BUFFER_SIZE);

  /* Route ADC1 to DMA1 Channel 1 using DMAMUX */
  LL_DMAMUX_SetRequestID(DMAMUX1, LL_DMAMUX_CHANNEL_0, LL_DMAMUX_REQ_ADC1);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

  /* Start ADC1 Regular continuous conversion */
  LL_ADC_REG_StartConversion(ADC1);

  /*
   * Enable Injected End of Conversion (JEOC) Interrupt on Master ADC1:
   * This ISR is the heart of the Field-Oriented Control loop.
   */
  LL_ADC_ClearFlag_JEOC(ADC1);
  LL_ADC_EnableIT_JEOC(ADC1);

  /* Configure NVIC for ADC1_2_IRQn: Highest priority for motor control */
  NVIC_SetPriority(ADC1_2_IRQn, 0U);
  NVIC_EnableIRQ(ADC1_2_IRQn);
}

/**
 * @brief  Initializes Hardware CORDIC Accelerator Coprocessor.
 *         Configured for 24-bit precision, Sine/Cosine calculation in parallel.
 * @retval None
 */
void Board_CORDIC_Init(void) {
  /* Enable CORDIC Peripheral Clock */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);

  /*
   * CORDIC Configuration:
   * Function: Sine and Cosine
   * Precision: 6 cycles (equivalent to 24-bit accuracy, error < 2^-20)
   * Data size: 32-bit (Q31 format)
   * Number of arguments: 1 (Angle in Q31: [-PI, PI] mapped to [-1, 1))
   * Number of results: 2 (First read Cosine, Second read Sine)
   */
  LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_COSINE,
                   LL_CORDIC_PRECISION_6CYCLES, LL_CORDIC_SCALE_0,
                   LL_CORDIC_NBWRITE_1, LL_CORDIC_NBREAD_2,
                   LL_CORDIC_INSIZE_32BITS, LL_CORDIC_OUTSIZE_32BITS);
}

/**
 * @brief  Initializes USART1 for non-blocking CLI and telemetry streaming.
 * @retval None
 */
void Board_USART1_Init(void) {
  /* Enable USART1 peripheral clock */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_USART_InitTypeDef usart_init = {0};
  usart_init.PrescalerValue = LL_USART_PRESCALER_DIV1;
  usart_init.BaudRate = CLI_USART_BAUDRATE;
  usart_init.DataWidth = LL_USART_DATAWIDTH_8B;
  usart_init.StopBits = LL_USART_STOPBITS_1;
  usart_init.Parity = LL_USART_PARITY_NONE;
  usart_init.TransferDirection = LL_USART_DIRECTION_TX_RX;
  usart_init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  usart_init.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &usart_init);

  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
}

/**
 * @brief  Initializes FDCAN1 interface for DroneCAN and telemetry frames.
 * @retval None
 */
void Board_FDCAN1_Init(void) {
  /* Enable FDCAN peripheral clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_FDCAN);

  /* FDCAN initialization is handled in canfd_interface.c via HAL/LL */
}

/**
 * @brief  Initializes TIM2 for high-speed DShot throttle capture.
 * @retval None
 */
void Board_DShot_Timer_Init(void) {
  /* Enable TIM2 clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* DShot timer initialization detailed in dshot_decoder.c */
}

/**
 * @brief  Master initialization calling all peripheral setup routines in order.
 * @retval None
 */
void Board_Init_All(void) {
  /* 1. System Clocks */
  Board_SystemClock_Config();

  /* 2. DWT Cycle Counter for high-resolution timing */
  DWT_Init();

  /* 3. GPIO Pin Mappings */
  Board_GPIO_Init();

  /* 4. Hardware CORDIC Coprocessor */
  Board_CORDIC_Init();

  /* 5. Internal OPAMPs */
  Board_OPAMP_Init();

  /* 6. Hardware OCP Comparator */
  Board_COMP_OCP_Init();

  /* 7. Dual ADC1/ADC2 Simultaneous Injected & Regular DMA */
  Board_ADC_Dual_Init();

  /* 8. TIM1 Complementary PWM Inverter Timer */
  Board_TIM1_PWM_Init();

  /* 9. Serial Telemetry USART1 */
  Board_USART1_Init();

  /* 10. CAN FD */
  Board_FDCAN1_Init();

  /* 11. DShot / PWM Input Capture Timer */
  Board_DShot_Timer_Init();

  /* Start with power LED ON */
  LED1_GREEN_ON();
  LED2_BLUE_OFF();
}

/**
 * @brief  ADC1 and ADC2 Injected Conversion Complete Interrupt Handler.
 *         This ISR is triggered at 24 kHz at the exact center of low-side
 *         MOSFET conduction. Executes the FOC algorithm in <8 microseconds.
 * @retval None
 */
void ADC1_2_IRQHandler(void) {
  /* Check Injected End of Conversion flag */
  if (LL_ADC_IsActiveFlag_JEOC(ADC1) != 0U) {
    /* Clear the JEOC flag */
    LL_ADC_ClearFlag_JEOC(ADC1);

    /* Invoke the core Field-Oriented Control loop callback */
    FOC_ADC_Injected_Callback();
  }
}
