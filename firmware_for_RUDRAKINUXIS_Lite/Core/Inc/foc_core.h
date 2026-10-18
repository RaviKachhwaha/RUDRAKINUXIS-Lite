/**
 ******************************************************************************
 * @file           : foc_core.h
 * @brief          : Production-grade Field-Oriented Control (FOC) engine.
 *                   ADC injected sampling, current offset auto-calibration,
 *                   Park/Clarke transforms, decoupling, and high-speed ISR.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 RUDRAKINUXIS Project.
 * All rights reserved.
 *
 ******************************************************************************
 */

#ifndef __FOC_CORE_H
#define __FOC_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "board_config.h"
#include "foc_math.h"
#include "observer_smo.h"

/* Operational FOC States */
typedef enum {
  FOC_STATE_UNINITIALIZED = 0,
  FOC_STATE_CALIBRATION,
  FOC_STATE_DISARMED,
  FOC_STATE_ARMED,
  FOC_STATE_ALIGN,
  FOC_STATE_STARTUP_RAMP,
  FOC_STATE_CLOSED_LOOP,
  FOC_STATE_FAULT
} foc_state_t;

/* Motor Configuration Structure */
typedef struct {
  float rs_ohms;         /**< Phase resistance in Ohms */
  float ls_henries;      /**< Phase inductance in Henries */
  float flux_linkage_wb; /**< PM flux linkage (Weber) */
  uint8_t pole_pairs;    /**< Number of magnetic pole pairs */
  float max_current_a;   /**< Maximum permissible continuous phase current */
} foc_motor_params_t;

/* Master FOC Controller Structure */
typedef struct {
  /* State Machine */
  volatile foc_state_t state;
  bool is_armed;

  /* Current Sensing & Calibrated ADC Offsets */
  uint32_t calib_samples_count;
  uint32_t calib_accum_u;
  uint32_t calib_accum_v;
  float offset_u;
  float offset_v;
  bool is_calibrated;

  /* Measured Current Vectors */
  foc_abc_t i_abc;
  foc_alphabeta_t i_alphabeta;
  foc_dq_t i_dq;

  /* Target Current Commands */
  float target_id_a; /**< Target d-axis current (0 for MTPA on surface PM) */
  float target_iq_a; /**< Target q-axis current (proportional to throttle) */
  float max_iq_allowed_a; /**< Dynamically clamped by thermal & battery
                             supervisor */

  /* Current PI Regulators */
  foc_pi_controller_t pi_d;
  foc_pi_controller_t pi_q;

  /* Feedforward Decoupling Terms */
  float v_d_ff;
  float v_q_ff;

  /* Voltage Commands */
  foc_dq_t v_dq;
  foc_alphabeta_t v_alphabeta;

  /* Timer Compare Values */
  uint16_t pwm_ccr1;
  uint16_t pwm_ccr2;
  uint16_t pwm_ccr3;
  uint8_t svpwm_sector;

  /* Power Stage Feedback */
  float v_bus;        /**< Filtered DC bus voltage */
  float board_temp_c; /**< Measured power stage temperature */

  /* Angle & Observer */
  float rotor_angle_rad;
  observer_smo_t smo;
  foc_startup_t startup;
  foc_motor_params_t motor;

  /* Performance Profiling */
  uint32_t isr_cycles_last;
  uint32_t isr_cycles_max;
  float isr_duration_us;
} foc_controller_t;

/* Global Controller Instance */
extern foc_controller_t g_foc;

/*============================================================================*/
/*                      FUNCTION PROTOTYPES                                   */
/*============================================================================*/

/**
 * @brief  Initializes the FOC controller, PI loops, and motor parameters.
 * @param  foc Pointer to controller struct
 * @retval None
 */
void FOC_Core_Init(foc_controller_t *foc);

/**
 * @brief  Initiates automatic zero-current ADC offset calibration.
 * @param  foc Pointer to controller struct
 * @retval None
 */
void FOC_Core_StartCalibration(foc_controller_t *foc);

/**
 * @brief  Arms the ESC, transitioning from DISARMED to active motor control.
 * @param  foc Pointer to controller struct
 * @retval None
 */
void FOC_Core_Arm(foc_controller_t *foc);

/**
 * @brief  Disarms the ESC, clearing gate outputs and resetting regulators.
 * @param  foc Pointer to controller struct
 * @retval None
 */
void FOC_Core_Disarm(foc_controller_t *foc);

/**
 * @brief  Sets target torque current Iq (saturated to safe operational limits).
 * @param  foc         Pointer to controller struct
 * @param  target_iq_a Commanded current in Amperes
 * @retval None
 */
void FOC_Core_SetTargetCurrent(foc_controller_t *foc, float target_iq_a);

/**
 * @brief  Emergency shutdown triggered by safety supervisor or fault.
 * @param  foc Pointer to controller struct
 * @retval None
 */
void FOC_Core_Fault_Trip(foc_controller_t *foc);

/**
 * @brief  High-Speed 24 kHz FOC Loop callback.
 *         Called directly from ADC1_2_IRQHandler at the center of low-side PWM
 * on-time. Must complete in under 8 microseconds.
 * @retval None
 */
void FOC_ADC_Injected_Callback(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_CORE_H */
