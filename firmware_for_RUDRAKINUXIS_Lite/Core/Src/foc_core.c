/**
  ******************************************************************************
  * @file           : foc_core.c
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

#include "foc_core.h"

/* Global FOC Controller Instance */
foc_controller_t g_foc;

/* Number of ADC samples to average for zero-current calibration */
#define FOC_CALIB_SAMPLES               (2048U)

/**
  * @brief  Initializes the FOC controller, PI loops, and motor parameters.
  */
void FOC_Core_Init(foc_controller_t *foc)
{
    memset(foc, 0, sizeof(foc_controller_t));

    /* Initialize default motor parameters (Typical 6S 600W Brushless Multirotor Motor) */
    foc->motor.rs_ohms          = SMO_DEFAULT_RS_OHMS;
    foc->motor.ls_henries       = SMO_DEFAULT_LS_HENRIES;
    foc->motor.flux_linkage_wb  = 0.00085f; /* 850 uWb PM flux */
    foc->motor.pole_pairs       = 7U;       /* 14-pole rotor */
    foc->motor.max_current_a    = CURRENT_MAX_PEAK_A;

    /* Initialize limits */
    foc->max_iq_allowed_a       = CURRENT_RATED_CONT_A;
    foc->v_bus                  = VBUS_NOMINAL_V;

    /*
     * Initialize Current PI Controllers:
     * Bandwidth target: ~1500 rad/s
     * Kp = Ls * omega_bw = 15uH * 1500 = ~0.025 to 0.15 depending on scaling.
     * Ki = Rs * omega_bw = 0.025 * 1500 = ~37.5 to 150.0.
     * Clamped to +- (VBUS / sqrt(3)) Volts.
     */
    float max_v = VBUS_NOMINAL_V * FOC_ONE_OVER_SQRT3;
    FOC_PI_Init(&foc->pi_d, 0.15f, 150.0f, -max_v, max_v, FOC_LOOP_PERIOD_SEC);
    FOC_PI_Init(&foc->pi_q, 0.15f, 150.0f, -max_v, max_v, FOC_LOOP_PERIOD_SEC);

    /* Initialize Sliding Mode Observer & Startup Engine */
    Observer_SMO_Init(&foc->smo,
                      foc->motor.rs_ohms,
                      foc->motor.ls_henries,
                      foc->motor.pole_pairs,
                      FOC_LOOP_PERIOD_SEC);

    FOC_Startup_Init(&foc->startup, 4.0f); /* 4.0A startup ramp current */

    /* Begin automatic zero-current calibration */
    FOC_Core_StartCalibration(foc);
}

/**
  * @brief  Initiates automatic zero-current ADC offset calibration.
  *         Disables gate driver outputs while measuring baseline offsets.
  */
void FOC_Core_StartCalibration(foc_controller_t *foc)
{
    /* Ensure gates are completely disabled during calibration */
    Board_EmergencyStop_Hardware();

    foc->calib_samples_count = 0U;
    foc->calib_accum_u       = 0U;
    foc->calib_accum_v       = 0U;
    foc->is_calibrated       = false;
    foc->state               = FOC_STATE_CALIBRATION;
}

/**
  * @brief  Arms the ESC, transitioning from DISARMED to active motor control.
  */
void FOC_Core_Arm(foc_controller_t *foc)
{
    if ((foc->state == FOC_STATE_DISARMED) && foc->is_calibrated)
    {
        /* Reset controllers and observers */
        FOC_PI_Reset(&foc->pi_d);
        FOC_PI_Reset(&foc->pi_q);
        Observer_SMO_Reset(&foc->smo);
        FOC_Startup_Reset(&foc->startup);

        foc->target_id_a     = 0.0f;
        foc->target_iq_a     = 0.0f;
        foc->rotor_angle_rad = 0.0f;

        /* Enable Gate Driver hardware */
        Board_EnableGates_Hardware();

        foc->is_armed = true;
        foc->state    = FOC_STATE_ARMED;
    }
}

/**
  * @brief  Disarms the ESC, clearing gate outputs and resetting regulators.
  */
void FOC_Core_Disarm(foc_controller_t *foc)
{
    /* Clear PWM duty outputs immediately */
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;

    /* Disable gate driver outputs */
    Board_EmergencyStop_Hardware();

    foc->is_armed        = false;
    foc->target_iq_a     = 0.0f;
    foc->target_id_a     = 0.0f;
    foc->rotor_angle_rad = 0.0f;

    FOC_PI_Reset(&foc->pi_d);
    FOC_PI_Reset(&foc->pi_q);
    Observer_SMO_Reset(&foc->smo);
    FOC_Startup_Reset(&foc->startup);

    if (foc->state != FOC_STATE_FAULT)
    {
        foc->state = FOC_STATE_DISARMED;
    }
}

/**
  * @brief  Sets target torque current Iq (saturated to safe operational limits).
  */
void FOC_Core_SetTargetCurrent(foc_controller_t *foc, float target_iq_a)
{
    /* Saturated to dynamic thermal/voltage limit */
    if (target_iq_a > foc->max_iq_allowed_a)
    {
        target_iq_a = foc->max_iq_allowed_a;
    }
    else if (target_iq_a < 0.0f)
    {
        /* Unidirectional ESC mode: no regenerative braking on throttle down */
        target_iq_a = 0.0f;
    }

    if (foc->state == FOC_STATE_CLOSED_LOOP)
    {
        foc->target_iq_a = target_iq_a;
    }
}

/**
  * @brief  Emergency shutdown triggered by safety supervisor or fault.
  */
void FOC_Core_Fault_Trip(foc_controller_t *foc)
{
    /* Immediate hardware gate cutoff */
    Board_EmergencyStop_Hardware();

    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;

    foc->is_armed    = false;
    foc->target_iq_a = 0.0f;
    foc->target_id_a = 0.0f;
    foc->state       = FOC_STATE_FAULT;
}

/**
  * @brief  High-Speed 24 kHz FOC Loop callback.
  *         Called directly from ADC1_2_IRQHandler at the center of low-side PWM on-time.
  *         Deterministic execution profile: < 8 microseconds (< 1360 cycles @ 170 MHz).
  */
void FOC_ADC_Injected_Callback(void)
{
    uint32_t start_cycles = DWT_GetCycles();

    /*
     * 1. Simultaneous Phase Current Acquisition:
     *    Injected conversions on ADC1 (Phase U) and ADC2 (Phase V)
     *    triggered precisely at TIM1 counter valley (underflow).
     */
    uint16_t raw_adc_u = (uint16_t)ADC1->JDR1;
    uint16_t raw_adc_v = (uint16_t)ADC2->JDR1;

    /* Handle Calibration State */
    if (g_foc.state == FOC_STATE_CALIBRATION)
    {
        g_foc.calib_accum_u += raw_adc_u;
        g_foc.calib_accum_v += raw_adc_v;
        g_foc.calib_samples_count++;

        if (g_foc.calib_samples_count >= FOC_CALIB_SAMPLES)
        {
            g_foc.offset_u = (float)g_foc.calib_accum_u / (float)FOC_CALIB_SAMPLES;
            g_foc.offset_v = (float)g_foc.calib_accum_v / (float)FOC_CALIB_SAMPLES;

            /* Verify calibration offsets are centered around 1.65V (approx 2048 +- 300 counts) */
            if ((g_foc.offset_u > 1700.0f) && (g_foc.offset_u < 2400.0f) &&
                (g_foc.offset_v > 1700.0f) && (g_foc.offset_v < 2400.0f))
            {
                g_foc.is_calibrated = true;
                g_foc.state         = FOC_STATE_DISARMED;
            }
            else
            {
                /* Sensor offset error -> Trip fault */
                FOC_Core_Fault_Trip(&g_foc);
            }
        }
        return;
    }

    /* Update DC bus voltage from continuous background DMA regular conversions */
    g_foc.v_bus = (float)g_adc_regular_buffer[ADC_IDX_VBUS] * VBUS_SENSE_SCALE_FACTOR;
    if (g_foc.v_bus < 10.0f)
    {
        g_foc.v_bus = VBUS_NOMINAL_V; /* Fallback safe value during bench test */
    }

    /*
     * 2. Phase Current Reconstruction:
     *    I_u = (ADC_u - Offset_u) * Scale
     *    I_v = (ADC_v - Offset_v) * Scale
     *    Kirchhoff's Current Law: I_u + I_v + I_w = 0 -> I_w = -(I_u + I_v)
     */
    g_foc.i_abc.a = ((float)raw_adc_u - g_foc.offset_u) * CURRENT_SENSE_SCALE_FACTOR;
    g_foc.i_abc.b = ((float)raw_adc_v - g_foc.offset_v) * CURRENT_SENSE_SCALE_FACTOR;
    g_foc.i_abc.c = -(g_foc.i_abc.a + g_foc.i_abc.b);

    /* 3. Clarke Transformation: 3-Phase to Stationary Orthogonal (I_alpha, I_beta) */
    FOC_Clarke_Transform(&g_foc.i_abc, &g_foc.i_alphabeta);

    /* Check if system is disarmed or faulted */
    if ((g_foc.state == FOC_STATE_DISARMED) || (g_foc.state == FOC_STATE_FAULT))
    {
        TIM1->CCR1 = 0U;
        TIM1->CCR2 = 0U;
        TIM1->CCR3 = 0U;
        return;
    }

    /*
     * 4. Rotor Angle Estimation & Startup State Machine:
     *    Runs Sliding Mode Observer and I-F ramp handoff controller.
     */
    if (g_foc.state == FOC_STATE_ARMED)
    {
        /* Start I-F alignment and ramp trajectory */
        g_foc.state = FOC_STATE_ALIGN;
        FOC_Startup_Reset(&g_foc.startup);
    }

    /* Step the startup engine */
    g_foc.rotor_angle_rad = FOC_Startup_Step(&g_foc.startup, &g_foc.smo, FOC_LOOP_PERIOD_SEC);

    /* Run SMO observer update with previous voltage commands and current feedback */
    Observer_SMO_Update(&g_foc.smo,
                        g_foc.v_alphabeta.alpha,
                        g_foc.v_alphabeta.beta,
                        g_foc.i_alphabeta.alpha,
                        g_foc.i_alphabeta.beta);

    /* Transition states based on startup progress */
    if (g_foc.startup.is_closed_loop)
    {
        g_foc.state = FOC_STATE_CLOSED_LOOP;
        g_foc.rotor_angle_rad = g_foc.smo.theta_compensated;
    }
    else
    {
        if (g_foc.startup.state == STARTUP_STATE_ALIGN)
        {
            g_foc.state = FOC_STATE_ALIGN;
        }
        else
        {
            g_foc.state = FOC_STATE_STARTUP_RAMP;
        }
        /* Throttle setpoint driven by startup ramp */
        g_foc.target_iq_a = g_foc.startup.current_target_iq_a;
    }

    /*
     * 5. Hardware CORDIC Fast Sine / Cosine:
     *    Computes sin(theta) and cos(theta) in 6 clock cycles.
     */
    float sin_theta, cos_theta;
    FOC_Cordic_SinCos(g_foc.rotor_angle_rad, &sin_theta, &cos_theta);

    /* 6. Park Transformation: Stationary to Rotating Synchronous (I_d, I_q) */
    FOC_Park_Transform(&g_foc.i_alphabeta, sin_theta, cos_theta, &g_foc.i_dq);

    /*
     * 7. Cross-Coupling Feedforward Decoupling:
     *    V_d_ff = -omega_e * Ls * I_q
     *    V_q_ff =  omega_e * (Ls * I_d + lambda_pm)
     */
    float omega_e = g_foc.smo.omega_elec_rad_s;
    g_foc.v_d_ff = -omega_e * g_foc.motor.ls_henries * g_foc.i_dq.q;
    g_foc.v_q_ff =  omega_e * ((g_foc.motor.ls_henries * g_foc.i_dq.d) + g_foc.motor.flux_linkage_wb);

    /* Update dynamic controller saturation limits based on current V_bus */
    float v_max = g_foc.v_bus * FOC_ONE_OVER_SQRT3;
    g_foc.pi_d.out_max =  v_max;
    g_foc.pi_d.out_min = -v_max;
    g_foc.pi_q.out_max =  v_max;
    g_foc.pi_q.out_min = -v_max;

    /*
     * 8. Dual PI Current Controllers:
     *    I_d controller forces d-axis current to 0 (MTPA).
     *    I_q controller tracks commanded torque current.
     */
    float v_d_pi = FOC_PI_Update(&g_foc.pi_d, g_foc.target_id_a, g_foc.i_dq.d);
    float v_q_pi = FOC_PI_Update(&g_foc.pi_q, g_foc.target_iq_a, g_foc.i_dq.q);

    g_foc.v_dq.d = v_d_pi + g_foc.v_d_ff;
    g_foc.v_dq.q = v_q_pi + g_foc.v_q_ff;

    /* 9. Voltage Vector Circle Limitation: Clamps sqrt(Vd^2 + Vq^2) <= Vbus / sqrt(3) */
    FOC_Voltage_Circle_Limitation(&g_foc.v_dq, g_foc.v_bus);

    /* 10. Inverse Park Transformation: Rotating to Stationary (V_alpha, V_beta) */
    FOC_InvPark_Transform(&g_foc.v_dq, sin_theta, cos_theta, &g_foc.v_alphabeta);

    /*
     * 11. Space Vector Modulation (SVPWM):
     *     Generates 7-segment center-aligned compare values (CCR1, CCR2, CCR3).
     */
    g_foc.svpwm_sector = FOC_SVPWM_Generate(&g_foc.v_alphabeta,
                                             g_foc.v_bus,
                                             PWM_TIMER_ARR_VALUE,
                                             &g_foc.pwm_ccr1,
                                             &g_foc.pwm_ccr2,
                                             &g_foc.pwm_ccr3);

    /* 12. Load Inverter Timer Compare Registers Directly */
    TIM1->CCR1 = g_foc.pwm_ccr1;
    TIM1->CCR2 = g_foc.pwm_ccr2;
    TIM1->CCR3 = g_foc.pwm_ccr3;

    /* 13. Execution Profiling */
    uint32_t elapsed_cycles = DWT_GetCycles() - start_cycles;
    g_foc.isr_cycles_last = elapsed_cycles;
    if (elapsed_cycles > g_foc.isr_cycles_max)
    {
        g_foc.isr_cycles_max = elapsed_cycles;
    }
    g_foc.isr_duration_us = (float)elapsed_cycles / 170.0f;
}
