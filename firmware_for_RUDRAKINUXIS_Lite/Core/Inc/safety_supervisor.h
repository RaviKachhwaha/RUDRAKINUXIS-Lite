/**
  ******************************************************************************
  * @file           : safety_supervisor.h
  * @brief          : Deterministic 1 kHz Safety Supervisor and Protection Matrix.
  *                   Monitors OCP, OVP, UVLO, NTC temperature derating, rotor stall,
  *                   signal loss watchdog, and status LED patterns.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 RUDRAKINUXIS Project.
  * All rights reserved.
  *
  ******************************************************************************
  */

#ifndef __SAFETY_SUPERVISOR_H
#define __SAFETY_SUPERVISOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Safety Fault Bitfield Flags */
#define FAULT_FLAG_NONE                 (0x0000U)
#define FAULT_FLAG_SW_OCP               (1U << 0)   /**< Software current limit exceeded >5ms */
#define FAULT_FLAG_HW_OCP               (1U << 1)   /**< Hardware COMP1/TIM1 BKIN break event */
#define FAULT_FLAG_OVP                  (1U << 2)   /**< DC bus overvoltage > 26.5V */
#define FAULT_FLAG_UVLO                 (1U << 3)   /**< DC bus undervoltage < 18.0V */
#define FAULT_FLAG_OTP                  (1U << 4)   /**< Power stage overtemperature > 105C */
#define FAULT_FLAG_ROTOR_LOCK           (1U << 5)   /**< Motor rotor stalled under power > 250ms */
#define FAULT_FLAG_SIGNAL_TIMEOUT       (1U << 6)   /**< Throttle signal missing > 100ms */
#define FAULT_FLAG_CALIB_ERROR          (1U << 7)   /**< Sensor offset calibration failure */

/* Safety Supervisor State Structure */
typedef struct
{
    uint32_t active_faults;
    float    measured_temp_c;
    float    derated_current_limit_a;

    /* Fault Integration Counters (milliseconds) */
    uint16_t ocp_counter_ms;
    uint16_t ovp_counter_ms;
    uint16_t uvlo_counter_ms;
    uint16_t stall_counter_ms;

    /* Signal Watchdog */
    uint32_t last_valid_throttle_tick;
    bool     signal_loss_triggered;

    /* LED Animation Timers */
    uint16_t led_timer_ms;
} safety_supervisor_t;

extern safety_supervisor_t g_safety;

/*============================================================================*/
/*                      FUNCTION PROTOTYPES                                   */
/*============================================================================*/

/**
  * @brief  Initializes the Safety Supervisor and clears fault states.
  * @retval None
  */
void Safety_Supervisor_Init(void);

/**
  * @brief  Deterministic 1 kHz safety task. Called periodically from SysTick.
  *         Executes health checks, fault tripping, thermal derating, and LED updates.
  * @retval None
  */
void Safety_Supervisor_Task_1kHz(void);

/**
  * @brief  Converts raw NTC thermistor ADC counts into temperature in Celsius
  *         using the Beta parameter equation.
  * @param  adc_raw 12-bit ADC raw count from NTC channel
  * @retval Temperature in degrees Celsius
  */
float Safety_Calculate_NTC_Temperature(uint16_t adc_raw);

/**
  * @brief  Clears all active latching faults (if safety criteria are met).
  * @retval true if faults were successfully cleared, false if fault condition persists.
  */
bool Safety_ClearFaults(void);

/**
  * @brief  Returns active fault bitfield.
  * @retval Bitfield of active faults
  */
static inline uint32_t Safety_GetFaults(void)
{
    return g_safety.active_faults;
}

#ifdef __cplusplus
}
#endif

#endif /* __SAFETY_SUPERVISOR_H */
