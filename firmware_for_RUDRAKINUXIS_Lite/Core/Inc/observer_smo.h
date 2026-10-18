/**
 ******************************************************************************
 * @file           : observer_smo.h
 * @brief          : Sensorless Rotor Position Observer: Sliding Mode Observer
 * (SMO) with Phase-Locked Loop (PLL) and I-F Open-Loop Startup Engine.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 RUDRAKINUXIS Project.
 * All rights reserved.
 *
 ******************************************************************************
 */

#ifndef __OBSERVER_SMO_H
#define __OBSERVER_SMO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "foc_math.h"

/* Observer Parameters & Limits */
#define SMO_DEFAULT_RS_OHMS                                                    \
  (0.025f) /**< Typical 6S drone BLDC stator phase resistance */
#define SMO_DEFAULT_LS_HENRIES                                                 \
  (0.000015f) /**< Typical 6S drone BLDC phase inductance (15 uH) */
#define SMO_DEFAULT_K_SLIDE (10.0f) /**< Sliding mode gain K_smo (Volts) */
#define SMO_DEFAULT_CUTOFF_HZ                                                  \
  (1200.0f)                   /**< Back-EMF LPF cutoff frequency in Hz */
#define SMO_PLL_KP (500.0f)   /**< Angle tracking PLL proportional gain */
#define SMO_PLL_KI (50000.0f) /**< Angle tracking PLL integral gain */

#define SMO_SWITCHOVER_ELEC_RAD_S                                              \
  (150.0f) /**< Speed threshold to transition to closed-loop (~1430 eRPM) */
#define SMO_MIN_BEMF_AMPLITUDE_V                                               \
  (0.60f) /**< Minimum back-EMF magnitude required for handoff */
#define SMO_ALIGNMENT_TIME_S                                                   \
  (0.080f) /**< Rotor initial d-axis alignment duration in seconds */
#define SMO_RAMP_ACCEL_RAD_S2                                                  \
  (800.0f) /**< Open-loop I-F acceleration rate (rad/s^2) */
#define SMO_BLEND_DURATION_S                                                   \
  (0.040f) /**< Angle blending transition duration in seconds */

/**
 * @brief Startup State Machine Enum
 */
typedef enum {
  STARTUP_STATE_IDLE = 0,
  STARTUP_STATE_ALIGN,
  STARTUP_STATE_RAMP,
  STARTUP_STATE_BLEND,
  STARTUP_STATE_CLOSED_LOOP,
  STARTUP_STATE_FAULT
} foc_startup_state_t;

/**
 * @brief Sliding Mode Observer (SMO) State Structure
 */
typedef struct {
  /* Motor Physical Parameters */
  float rs;               /**< Stator phase resistance (Ohms) */
  float ls;               /**< Stator phase inductance (Henries) */
  float k_smo;            /**< Sliding control gain */
  float boundary_epsilon; /**< Boundary layer thickness to avoid chattering */

  /* Discretization Coefficients */
  float f_coeff;       /**< 1 - (Rs * Ts / Ls) */
  float g_coeff;       /**< Ts / Ls */
  float lpf_alpha;     /**< First-order LPF coefficient: (omega_c * Ts) / (1 +
                          omega_c * Ts) */
  float cutoff_rad_s;  /**< LPF cutoff frequency in rad/s */
  float sample_time_s; /**< Sampling time Ts = 1 / 24000 s */

  /* Observer Internal States */
  float i_alpha_est;    /**< Estimated stationary alpha current */
  float i_beta_est;     /**< Estimated stationary beta current */
  float z_alpha;        /**< Raw switching correction signal alpha */
  float z_beta;         /**< Raw switching correction signal beta */
  float e_alpha_est;    /**< Filtered back-EMF alpha */
  float e_beta_est;     /**< Filtered back-EMF beta */
  float bemf_magnitude; /**< Filtered back-EMF vector magnitude */

  /* PLL Angle & Speed Tracker */
  float pll_kp;            /**< PLL Proportional gain */
  float pll_ki;            /**< PLL Integral gain */
  float pll_integrator;    /**< PLL Speed integrator */
  float theta_raw;         /**< Raw observer angle */
  float theta_compensated; /**< Delay-compensated electrical rotor angle [-PI,
                              +PI] */
  float omega_elec_rad_s;  /**< Estimated electrical angular velocity (rad/s) */
  float rpm_elec;          /**< Estimated electrical RPM */
  float rpm_mech;     /**< Estimated mechanical RPM (based on pole pairs) */
  uint8_t pole_pairs; /**< Motor pole pairs */
} observer_smo_t;

/**
 * @brief I-F Open-Loop Startup Engine Structure
 */
typedef struct {
  foc_startup_state_t state;
  float target_startup_current_a;
  float current_target_iq_a;
  float open_loop_angle_rad;
  float open_loop_speed_rad_s;
  float accel_rad_s2;
  float timer_s;
  float blend_ratio; /**< 0.0 = 100% open-loop, 1.0 = 100% SMO */
  uint16_t convergence_counter;
  bool is_closed_loop;
} foc_startup_t;

/*============================================================================*/
/*                      FUNCTION PROTOTYPES                                   */
/*============================================================================*/

/**
 * @brief  Initializes the Sliding Mode Observer with motor parameters.
 * @param  smo           Pointer to observer structure
 * @param  rs            Stator phase resistance in Ohms
 * @param  ls            Stator phase inductance in Henries
 * @param  pole_pairs    Motor pole pairs count
 * @param  sample_time_s Loop period Ts (e.g. 1 / 24000 s)
 * @retval None
 */
void Observer_SMO_Init(observer_smo_t *smo, float rs, float ls,
                       uint8_t pole_pairs, float sample_time_s);

/**
 * @brief  Resets observer internal states, currents, and PLL integrators.
 * @param  smo Pointer to observer structure
 * @retval None
 */
void Observer_SMO_Reset(observer_smo_t *smo);

/**
 * @brief  Executes a single cycle of the Sliding Mode Observer.
 *         Runs at 24 kHz inside ADC Injected conversion ISR.
 * @param  smo         Pointer to observer structure
 * @param  v_alpha     Stationary alpha voltage command
 * @param  v_beta      Stationary beta voltage command
 * @param  i_alpha_act Measured alpha current feedback
 * @param  i_beta_act  Measured beta current feedback
 * @retval Delay-compensated electrical rotor angle in radians [-PI, +PI]
 */
float Observer_SMO_Update(observer_smo_t *smo, float v_alpha, float v_beta,
                          float i_alpha_act, float i_beta_act);

/**
 * @brief  Initializes the open-loop I-F startup state machine.
 * @param  startup           Pointer to startup engine structure
 * @param  startup_current_a Startup current magnitude in Amperes
 * @retval None
 */
void FOC_Startup_Init(foc_startup_t *startup, float startup_current_a);

/**
 * @brief  Resets startup engine to IDLE state.
 * @param  startup Pointer to startup engine structure
 * @retval None
 */
void FOC_Startup_Reset(foc_startup_t *startup);

/**
 * @brief  Executes I-F startup trajectory and handles smooth handoff into
 * closed loop.
 * @param  startup Pointer to startup engine structure
 * @param  smo     Pointer to observer structure
 * @param  dt_s    Time step (FOC_LOOP_PERIOD_SEC)
 * @retval Selected electrical angle to be used by Park and Inverse Park
 * transforms
 */
float FOC_Startup_Step(foc_startup_t *startup, observer_smo_t *smo, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* __OBSERVER_SMO_H */
