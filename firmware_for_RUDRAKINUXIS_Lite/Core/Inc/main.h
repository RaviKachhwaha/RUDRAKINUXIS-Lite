/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   Production firmware for RUDRAKINUXIS-Lite 6S 600W FOC ESC.
 *                   Target: STM32G431CBU6 (Arm Cortex-M4F @ 170MHz)
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 RUDRAKINUXIS Project.
 * All rights reserved.
 *
 ******************************************************************************
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_comp.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_opamp.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_utils.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* System Tick & DWT Microsecond Timers --------------------------------------*/
static inline void DWT_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
}

static inline uint32_t DWT_GetCycles(void) { return DWT->CYCCNT; }

static inline uint32_t DWT_GetMicroseconds(void) {
  /* 170 MHz system clock: 170 cycles = 1 microsecond */
  return (DWT->CYCCNT / 170U);
}

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
