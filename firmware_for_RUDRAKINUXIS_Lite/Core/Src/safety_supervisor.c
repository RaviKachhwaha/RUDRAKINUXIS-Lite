/**
  ******************************************************************************
  * @file           : safety_supervisor.c
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

#include "safety_supervisor.h"
#include "board_config.h"
#include "foc_core.h"
#include "dshot_decoder.h"
#include "canfd_interface.h"

/* Global Safety Supervisor Instance */
safety_supervisor_t g_safety;

/**
  * @brief  Initializes the Safety Supervisor and clears fault states.
  */
void Safety_Supervisor_Init(void)
{
    memset(&g_safety, 0, sizeof(safety_supervisor_t));
    g_safety.active_faults            = FAULT_FLAG_NONE;
    g_safety.derated_current_limit_a  = CURRENT_MAX_PEAK_A;
    g_safety.last_valid_throttle_tick = HAL_GetTick();
}

/**
  * @brief  Converts raw NTC thermistor ADC counts into temperature in Celsius
  *         using the Beta parameter equation.
  */
float Safety_Calculate_NTC_Temperature(uint16_t adc_raw)
{
    /* Open circuit or short to ground detection */
    if (adc_raw <= 20U)
    {
        return 125.0f; /* Fail-safe trip high temperature */
    }
    if (adc_raw >= 4075U)
    {
        return -25.0f; /* Cold reading / open thermistor */
    }

    /*
     * Voltage Divider:
     * Pull-up resistor R_pu = 10 kOhms tied to 3.3V
     * NTC thermistor R_ntc tied to GND
     * V_adc = 3.3V * (R_ntc / (R_pu + R_ntc))
     * R_ntc = R_pu * (ADC_raw / (4095.0 - ADC_raw))
     */
    float adc_f = (float)adc_raw;
    float r_ntc = NTC_PULLUP_RESISTOR_OHMS * (adc_f / (ADC_FULL_SCALE_COUNTS - adc_f));

    /*
     * Beta Equation:
     * 1/T = 1/T0 + (1/Beta) * ln(R_ntc / R0)
     */
    float ln_ratio = logf(r_ntc / NTC_NOMINAL_RESISTANCE_OHMS);
    float inv_t_k = (1.0f / NTC_NOMINAL_TEMP_KELVIN) + (ln_ratio / NTC_BETA_COEFFICIENT);
    float temp_k  = 1.0f / inv_t_k;

    return (temp_k - 273.15f);
}

/**
  * @brief  Deterministic 1 kHz safety task. Called periodically from SysTick.
  *         Executes health checks, fault tripping, thermal derating, and LED updates.
  */
void Safety_Supervisor_Task_1kHz(void)
{
    /* 1. Update Measured NTC Temperature */
    uint16_t ntc_raw = g_adc_regular_buffer[ADC_IDX_NTC];
    g_safety.measured_temp_c = Safety_Calculate_NTC_Temperature(ntc_raw);
    g_foc.board_temp_c       = g_safety.measured_temp_c;

    /*
     * 2. Check Hardware Break Event:
     *    Triggered by internal COMP1 (analog over-current cutoff < 50 ns).
     */
    if (LL_TIM_IsActiveFlag_BRK(TIM1) != 0U)
    {
        LL_TIM_ClearFlag_BRK(TIM1);
        g_safety.active_faults |= FAULT_FLAG_HW_OCP;
        FOC_Core_Fault_Trip(&g_foc);
    }

    /*
     * 3. Software Over-Current Protection (OCP):
     *    Checks phase current magnitude against software trip limit (55A)
     *    with 5 ms persistence window.
     */
    float current_mag = sqrtf((g_foc.i_alphabeta.alpha * g_foc.i_alphabeta.alpha) +
                              (g_foc.i_alphabeta.beta  * g_foc.i_alphabeta.beta));

    if (current_mag > CURRENT_SW_OCP_LIMIT_A)
    {
        g_safety.ocp_counter_ms++;
        if (g_safety.ocp_counter_ms >= CURRENT_SW_OCP_PERSIST_MS)
        {
            g_safety.active_faults |= FAULT_FLAG_SW_OCP;
            FOC_Core_Fault_Trip(&g_foc);
        }
    }
    else
    {
        if (g_safety.ocp_counter_ms > 0U)
        {
            g_safety.ocp_counter_ms--;
        }
    }

    /*
     * 4. DC Bus Over-Voltage Protection (OVP):
     *    6S LiPo maximum safe margin is 26.5V. Trips if exceeded for > 10 ms.
     */
    if (g_foc.v_bus > VBUS_OVP_THRESHOLD_V)
    {
        g_safety.ovp_counter_ms++;
        if (g_safety.ovp_counter_ms >= 10U)
        {
            g_safety.active_faults |= FAULT_FLAG_OVP;
            FOC_Core_Fault_Trip(&g_foc);
        }
    }
    else
    {
        if (g_safety.ovp_counter_ms > 0U)
        {
            g_safety.ovp_counter_ms--;
        }
    }

    /*
     * 5. DC Bus Under-Voltage Lockout (UVLO):
     *    Critical LiPo cell depletion (< 3.0V / cell = 18.0V pack).
     *    Trips if low for > 20 ms while armed to protect cells and gate drivers.
     */
    if ((g_foc.v_bus < VBUS_UVLO_THRESHOLD_V) && g_foc.is_armed)
    {
        g_safety.uvlo_counter_ms++;
        if (g_safety.uvlo_counter_ms >= 20U)
        {
            g_safety.active_faults |= FAULT_FLAG_UVLO;
            FOC_Core_Fault_Trip(&g_foc);
        }
    }
    else
    {
        if (g_safety.uvlo_counter_ms > 0U)
        {
            g_safety.uvlo_counter_ms--;
        }
    }

    /*
     * 6. Thermal Derating & Critical Over-Temperature Protection (OTP):
     *    85°C to 105°C: Linear derating of maximum allowable Iq.
     *    > 105°C: Immediate gate cutoff.
     */
    if (g_safety.measured_temp_c >= TEMP_CRITICAL_SHUTDOWN_C)
    {
        g_safety.active_faults |= FAULT_FLAG_OTP;
        FOC_Core_Fault_Trip(&g_foc);
    }
    else if (g_safety.measured_temp_c >= TEMP_DERATING_START_C)
    {
        float derating_factor = (TEMP_CRITICAL_SHUTDOWN_C - g_safety.measured_temp_c) /
                                (TEMP_CRITICAL_SHUTDOWN_C - TEMP_DERATING_START_C);
        g_safety.derated_current_limit_a = 10.0f + (derating_factor * (CURRENT_MAX_PEAK_A - 10.0f));
    }
    else
    {
        g_safety.derated_current_limit_a = CURRENT_MAX_PEAK_A;
    }

    g_foc.max_iq_allowed_a = g_safety.derated_current_limit_a;

    /*
     * 7. Rotor Lock / Stall Detection:
     *    If motor is commanded with high torque (Iq > 15A) but electrical speed
     *    remains near zero (< 15 rad/s) for > 250 ms, disconnect inverter.
     */
    if (g_foc.is_armed && (fabsf(g_foc.target_iq_a) > 15.0f) &&
        (fabsf(g_foc.smo.omega_elec_rad_s) < 15.0f))
    {
        g_safety.stall_counter_ms++;
        if (g_safety.stall_counter_ms >= ROTOR_LOCK_TIMEOUT_MS)
        {
            g_safety.active_faults |= FAULT_FLAG_ROTOR_LOCK;
            FOC_Core_Fault_Trip(&g_foc);
        }
    }
    else
    {
        g_safety.stall_counter_ms = 0U;
    }

    /*
     * 8. Control Signal Loss Watchdog:
     *    Monitors reception timestamp of DShot and CAN frames.
     *    If no valid frame is received for > 100 ms while armed, disarm immediately.
     */
    uint32_t current_tick = HAL_GetTick();
    uint32_t dt_dshot = current_tick - g_dshot.last_packet_time_ms;
    uint32_t dt_can   = current_tick - g_can.last_rx_time_ms;

    if (g_foc.is_armed)
    {
        if ((dt_dshot > THROTTLE_SIGNAL_TIMEOUT_MS) && (dt_can > THROTTLE_SIGNAL_TIMEOUT_MS))
        {
            g_safety.signal_loss_triggered = true;
            g_safety.active_faults |= FAULT_FLAG_SIGNAL_TIMEOUT;
            FOC_Core_Disarm(&g_foc);
        }
        else
        {
            g_safety.signal_loss_triggered = false;
        }
    }

    /*
     * 9. Status LED Animations:
     *    LED1 (Green): Solid when 3.3V rail & supervisor are healthy.
     *    LED2 (Blue):
     *      - Active Fault: Rapid strobe (10 Hz).
     *      - Armed / Closed-Loop: Solid ON.
     *      - Disarmed: Gentle heartbeat (1 Hz, 50 ms flash).
     */
    LED1_GREEN_ON();

    g_safety.led_timer_ms = (g_safety.led_timer_ms + 1U) % 1000U;

    if (g_safety.active_faults != FAULT_FLAG_NONE)
    {
        /* Fault condition: 10 Hz blink */
        if ((g_safety.led_timer_ms % 100U) < 50U)
        {
            LED2_BLUE_ON();
        }
        else
        {
            LED2_BLUE_OFF();
        }
    }
    else if (g_foc.is_armed)
    {
        /* Armed: Solid ON */
        LED2_BLUE_ON();
    }
    else
    {
        /* Standby / Disarmed: 1 Hz heartbeat pulse */
        if (g_safety.led_timer_ms < 50U)
        {
            LED2_BLUE_ON();
        }
        else
        {
            LED2_BLUE_OFF();
        }
    }
}

/**
  * @brief  Clears all active latching faults (if safety criteria are met).
  */
bool Safety_ClearFaults(void)
{
    /* Only allow reset if underlying hardware conditions are cleared */
    if ((g_foc.v_bus >= VBUS_UVLO_THRESHOLD_V) &&
        (g_foc.v_bus <= VBUS_OVP_THRESHOLD_V) &&
        (g_safety.measured_temp_c < TEMP_DERATING_START_C))
    {
        g_safety.active_faults   = FAULT_FLAG_NONE;
        g_safety.ocp_counter_ms   = 0U;
        g_safety.ovp_counter_ms   = 0U;
        g_safety.uvlo_counter_ms  = 0U;
        g_safety.stall_counter_ms = 0U;

        if (g_foc.state == FOC_STATE_FAULT)
        {
            g_foc.state = FOC_STATE_DISARMED;
        }
        return true;
    }
    return false;
}
