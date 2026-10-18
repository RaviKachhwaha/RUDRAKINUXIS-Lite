/**
 ******************************************************************************
 * @file           : observer_smo.c
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

#include "observer_smo.h"
#include <math.h>

/**
 * @brief  Initializes the Sliding Mode Observer with motor parameters.
 */
void Observer_SMO_Init(observer_smo_t *smo, float rs, float ls,
                       uint8_t pole_pairs, float sample_time_s) {
  smo->rs = (rs > 0.001f) ? rs : SMO_DEFAULT_RS_OHMS;
  smo->ls = (ls > 0.000001f) ? ls : SMO_DEFAULT_LS_HENRIES;
  smo->pole_pairs =
      (pole_pairs > 0U) ? pole_pairs : 7U; /* Default 14-pole (7 pair) motor */
  smo->sample_time_s = sample_time_s;
  smo->k_smo = SMO_DEFAULT_K_SLIDE;

  /* Discretization terms: Euler forward approximation of dI/dt = -(Rs/Ls)*I +
   * (V - e)/Ls */
  smo->f_coeff = 1.0f - (smo->rs * smo->sample_time_s / smo->ls);
  smo->g_coeff = smo->sample_time_s / smo->ls;

  /*
   * Optimal discrete boundary layer:
   * In discrete time, to prevent single-step chattering and overshoot across
   * the sliding surface caused by G = Ts / Ls, epsilon is matched to G * k_smo.
   */
  smo->boundary_epsilon = smo->k_smo * smo->g_coeff;

  /* Low-pass filter for back-EMF reconstruction */
  smo->cutoff_rad_s = FOC_TWO_PI * SMO_DEFAULT_CUTOFF_HZ;
  smo->lpf_alpha = (smo->cutoff_rad_s * smo->sample_time_s) /
                   (1.0f + (smo->cutoff_rad_s * smo->sample_time_s));

  /* Angle tracking PLL gains */
  smo->pll_kp = SMO_PLL_KP;
  smo->pll_ki = SMO_PLL_KI;

  Observer_SMO_Reset(smo);
}

/**
 * @brief  Resets observer internal states, currents, and PLL integrators.
 */
void Observer_SMO_Reset(observer_smo_t *smo) {
  smo->i_alpha_est = 0.0f;
  smo->i_beta_est = 0.0f;
  smo->z_alpha = 0.0f;
  smo->z_beta = 0.0f;
  smo->e_alpha_est = 0.0f;
  smo->e_beta_est = 0.0f;
  smo->bemf_magnitude = 0.0f;
  smo->pll_integrator = 0.0f;
  smo->theta_raw = 0.0f;
  smo->theta_compensated = 0.0f;
  smo->omega_elec_rad_s = 0.0f;
  smo->rpm_elec = 0.0f;
  smo->rpm_mech = 0.0f;
}

/**
 * @brief  Executes a single cycle of the Sliding Mode Observer.
 *         Runs at 24 kHz inside ADC Injected conversion ISR in < 3
 * microseconds.
 */
float Observer_SMO_Update(observer_smo_t *smo, float v_alpha, float v_beta,
                          float i_alpha_act, float i_beta_act) {
  /* 1. Calculate current estimation error between observer model and measured
   * feedback */
  float err_alpha = smo->i_alpha_est - i_alpha_act;
  float err_beta = smo->i_beta_est - i_beta_act;

  /*
   * 2. Continuous Saturation function across boundary layer epsilon:
   *    Eliminates high-frequency chattering while preserving Lyapunov
   * stability.
   */
  float sat_alpha = err_alpha / smo->boundary_epsilon;
  if (sat_alpha > 1.0f)
    sat_alpha = 1.0f;
  if (sat_alpha < -1.0f)
    sat_alpha = -1.0f;

  float sat_beta = err_beta / smo->boundary_epsilon;
  if (sat_beta > 1.0f)
    sat_beta = 1.0f;
  if (sat_beta < -1.0f)
    sat_beta = -1.0f;

  smo->z_alpha = smo->k_smo * sat_alpha;
  smo->z_beta = smo->k_smo * sat_beta;

  /* 3. Discretized state update for estimated currents */
  smo->i_alpha_est = (smo->f_coeff * smo->i_alpha_est) +
                     (smo->g_coeff * (v_alpha - smo->z_alpha));

  smo->i_beta_est = (smo->f_coeff * smo->i_beta_est) +
                    (smo->g_coeff * (v_beta - smo->z_beta));

  /*
   * 4. 1st-Order Low-Pass Filter on switching signals to extract Back-EMF:
   *    z_alpha, z_beta contain back-EMF + high frequency switching noise.
   */
  smo->e_alpha_est += smo->lpf_alpha * (smo->z_alpha - smo->e_alpha_est);
  smo->e_beta_est += smo->lpf_alpha * (smo->z_beta - smo->e_beta_est);

  /* Back-EMF vector magnitude */
  smo->bemf_magnitude = sqrtf((smo->e_alpha_est * smo->e_alpha_est) +
                              (smo->e_beta_est * smo->e_beta_est));

  /*
   * 5. Phase-Locked Loop (PLL) Rotor Tracking:
   *    Motor BEMF equations:
   *      e_alpha = -E_mag * sin(theta)
   *      e_beta  =  E_mag * cos(theta)
   *    Phase Detector error:
   *      delta = -e_alpha * cos(theta_pll) - e_beta * sin(theta_pll)
   *            = E_mag * sin(theta - theta_pll) ~= E_mag * (theta - theta_pll)
   */
  float sin_pll, cos_pll;
  FOC_Cordic_SinCos(smo->theta_raw, &sin_pll, &cos_pll);

  float pll_err = (-smo->e_alpha_est * cos_pll) - (smo->e_beta_est * sin_pll);

  /* Normalize error by back-EMF amplitude if sufficiently large */
  if (smo->bemf_magnitude > 0.05f) {
    pll_err /= smo->bemf_magnitude;
  } else {
    pll_err = 0.0f;
  }

  /* PI speed tracking loop */
  smo->pll_integrator += (smo->pll_ki * smo->sample_time_s * pll_err);
  smo->omega_elec_rad_s = (smo->pll_kp * pll_err) + smo->pll_integrator;

  /* Integrate angular velocity to update raw PLL angle */
  smo->theta_raw += (smo->omega_elec_rad_s * smo->sample_time_s);
  smo->theta_raw = FOC_Angle_Normalize(smo->theta_raw);

  /*
   * 6. LPF Phase Lag Compensation:
   *    The 1st-order low-pass filter introduces a phase lag:
   *      delta_theta = atan(omega_elec / omega_cutoff)
   */
  float phase_delay = atan2f(smo->omega_elec_rad_s, smo->cutoff_rad_s);
  smo->theta_compensated = FOC_Angle_Normalize(smo->theta_raw + phase_delay);

  /* Calculate RPM values: 1 rad/s = 60 / (2*PI) = 9.54929658551 RPM */
  smo->rpm_elec = smo->omega_elec_rad_s * 9.54929658551f;
  smo->rpm_mech = smo->rpm_elec / (float)smo->pole_pairs;

  return smo->theta_compensated;
}

/**
 * @brief  Initializes the open-loop I-F startup state machine.
 */
void FOC_Startup_Init(foc_startup_t *startup, float startup_current_a) {
  startup->target_startup_current_a =
      (startup_current_a > 1.0f) ? startup_current_a : 4.0f;
  startup->accel_rad_s2 = SMO_RAMP_ACCEL_RAD_S2;
  FOC_Startup_Reset(startup);
}

/**
 * @brief  Resets startup engine to IDLE state.
 */
void FOC_Startup_Reset(foc_startup_t *startup) {
  startup->state = STARTUP_STATE_IDLE;
  startup->current_target_iq_a = 0.0f;
  startup->open_loop_angle_rad = 0.0f;
  startup->open_loop_speed_rad_s = 0.0f;
  startup->timer_s = 0.0f;
  startup->blend_ratio = 0.0f;
  startup->convergence_counter = 0U;
  startup->is_closed_loop = false;
}

/**
 * @brief  Executes I-F startup trajectory and handles smooth handoff into
 * closed loop.
 * @retval Electrical angle to use for FOC coordinate transforms.
 */
float FOC_Startup_Step(foc_startup_t *startup, observer_smo_t *smo,
                       float dt_s) {
  float selected_angle = 0.0f;

  switch (startup->state) {
  case STARTUP_STATE_IDLE:
    startup->state = STARTUP_STATE_ALIGN;
    startup->timer_s = 0.0f;
    startup->current_target_iq_a = 0.0f;
    startup->open_loop_angle_rad = 0.0f;
    startup->open_loop_speed_rad_s = 0.0f;
    selected_angle = 0.0f;
    break;

  case STARTUP_STATE_ALIGN:
    /*
     * Rotor Alignment Phase:
     * Hold angle at 0 (d-axis) and ramp current to alignment target.
     * Locks rotor firmly into place prior to motion.
     */
    startup->timer_s += dt_s;
    if (startup->current_target_iq_a < startup->target_startup_current_a) {
      startup->current_target_iq_a +=
          (startup->target_startup_current_a / SMO_ALIGNMENT_TIME_S) * dt_s;
    }

    startup->open_loop_angle_rad = 0.0f;
    selected_angle = 0.0f;

    if (startup->timer_s >= SMO_ALIGNMENT_TIME_S) {
      startup->state = STARTUP_STATE_RAMP;
      startup->timer_s = 0.0f;
    }
    break;

  case STARTUP_STATE_RAMP:
    /*
     * Open-Loop Acceleration Phase:
     * Linearly accelerate frequency omega_ol and integrate angle.
     * SMO runs in parallel, tracking rising back-EMF.
     */
    startup->open_loop_speed_rad_s += (startup->accel_rad_s2 * dt_s);
    startup->open_loop_angle_rad += (startup->open_loop_speed_rad_s * dt_s);
    startup->open_loop_angle_rad =
        FOC_Angle_Normalize(startup->open_loop_angle_rad);
    selected_angle = startup->open_loop_angle_rad;

    /* Check transition conditions: speed > threshold AND adequate BEMF
     * amplitude */
    if ((startup->open_loop_speed_rad_s >= SMO_SWITCHOVER_ELEC_RAD_S) &&
        (smo->bemf_magnitude >= SMO_MIN_BEMF_AMPLITUDE_V)) {
      /* Calculate angular difference between open-loop and observer */
      float angle_diff = fabsf(FOC_Angle_Normalize(
          smo->theta_compensated - startup->open_loop_angle_rad));

      /* If observer has locked on with < 20° phase error */
      if (angle_diff < 0.35f) {
        startup->convergence_counter++;
        if (startup->convergence_counter >=
            80U) /* 80 consecutive cycles (~3.3 ms) */
        {
          startup->state = STARTUP_STATE_BLEND;
          startup->blend_ratio = 0.0f;
        }
      } else {
        startup->convergence_counter = 0U;
      }
    }
    break;

  case STARTUP_STATE_BLEND:
    /*
     * Smooth Angle Blending Phase:
     * Linearly interpolate between open-loop trajectory and SMO angle
     * over SMO_BLEND_DURATION_S to prevent current transients.
     */
    startup->blend_ratio += (dt_s / SMO_BLEND_DURATION_S);
    if (startup->blend_ratio >= 1.0f) {
      startup->blend_ratio = 1.0f;
      startup->state = STARTUP_STATE_CLOSED_LOOP;
      startup->is_closed_loop = true;
      selected_angle = smo->theta_compensated;
    } else {
      /* Unwrapped interpolation */
      float delta_angle = FOC_Angle_Normalize(smo->theta_compensated -
                                              startup->open_loop_angle_rad);
      selected_angle = FOC_Angle_Normalize(
          startup->open_loop_angle_rad + (startup->blend_ratio * delta_angle));
    }
    break;

  case STARTUP_STATE_CLOSED_LOOP:
    /* Full Sensorless Field-Oriented Control */
    startup->is_closed_loop = true;
    selected_angle = smo->theta_compensated;
    break;

  case STARTUP_STATE_FAULT:
  default:
    startup->is_closed_loop = false;
    startup->current_target_iq_a = 0.0f;
    selected_angle = 0.0f;
    break;
  }

  return selected_angle;
}
