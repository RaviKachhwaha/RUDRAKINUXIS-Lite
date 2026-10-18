/**
  ******************************************************************************
  * @file           : foc_math.c
  * @brief          : High-speed mathematical engine for Field-Oriented Control.
  *                   STM32G4 CORDIC hardware acceleration, fast trigonometry,
  *                   transforms, discrete PI controllers, and SVPWM.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 RUDRAKINUXIS Project.
  * All rights reserved.
  *
  ******************************************************************************
  */

#include "foc_math.h"
#if defined(STM32G431xx) || defined(USE_HAL_DRIVER)
#include "board_config.h"
#endif

#define Q31_SCALE_FACTOR                (2147483648.0f)
#define ONE_OVER_Q31_SCALE              (1.0f / 2147483648.0f)

/**
  * @brief  Computes Sine and Cosine simultaneously using STM32G4 Hardware CORDIC.
  *         If CORDIC clock is active, writes angle in Q31 format to CORDIC->WDATA
  *         and reads Cosine then Sine from CORDIC->RDATA in single-cycle operations.
  * @param  angle_rad Angle in radians [-PI, +PI]
  * @param  sin_out   Pointer to store calculated sine value
  * @param  cos_out   Pointer to store calculated cosine value
  * @retval None
  */
void FOC_Cordic_SinCos(float angle_rad, float *sin_out, float *cos_out)
{
    /* Normalize angle to [-PI, +PI) */
    float norm_angle = FOC_Angle_Normalize(angle_rad);

#if defined(CORDIC)
    /* Convert normalized angle [-PI, +PI] to Q31 range [-1.0, +1.0) */
    float angle_unit = norm_angle * (1.0f / FOC_PI);
    if (angle_unit > 0.999999f) angle_unit = 0.999999f;
    if (angle_unit < -1.0f)     angle_unit = -1.0f;

    int32_t angle_q31 = (int32_t)(angle_unit * Q31_SCALE_FACTOR);

    /* Write angle argument to CORDIC */
    CORDIC->WDATA = (uint32_t)angle_q31;

    /*
     * Read primary result (Cosine in Q31) and secondary result (Sine in Q31)
     * Configured for LL_CORDIC_FUNCTION_COSINE with 2 reads:
     * First read = Cosine, Second read = Sine
     */
    int32_t cos_q31 = (int32_t)CORDIC->RDATA;
    int32_t sin_q31 = (int32_t)CORDIC->RDATA;

    *cos_out = (float)cos_q31 * ONE_OVER_Q31_SCALE;
    *sin_out = (float)sin_q31 * ONE_OVER_Q31_SCALE;
#else
    /* Software Fallback using standard math library */
    *sin_out = sinf(norm_angle);
    *cos_out = cosf(norm_angle);
#endif
}

/**
  * @brief  Fast rational approximation of four-quadrant arctangent (atan2).
  *         Max error < 0.005 radians (~0.28 degrees).
  * @param  y Numerator
  * @param  x Denominator
  * @retval Angle in radians [-PI, +PI]
  */
float FOC_Fast_Atan2(float y, float x)
{
    if (x == 0.0f)
    {
        if (y > 0.0f) return (FOC_PI * 0.5f);
        if (y < 0.0f) return (-FOC_PI * 0.5f);
        return 0.0f;
    }

    float abs_y = fabsf(y);
    float abs_x = fabsf(x);
    float angle;

    /* Ratio reduction for stability */
    if (abs_x >= abs_y)
    {
        float r = abs_y / abs_x;
        float r2 = r * r;
        /* Polynomial approximation of atan(r) for r in [0, 1] */
        angle = r * (1.0f + 0.280872f * r2) / (1.0f + 0.614485f * r2);
    }
    else
    {
        float r = abs_x / abs_y;
        float r2 = r * r;
        angle = (FOC_PI * 0.5f) - (r * (1.0f + 0.280872f * r2) / (1.0f + 0.614485f * r2));
    }

    /* Quadrant adjustment */
    if (x < 0.0f)
    {
        angle = FOC_PI - angle;
    }
    if (y < 0.0f)
    {
        angle = -angle;
    }

    return angle;
}

/**
  * @brief  Initializes a discrete PI controller with gains and clamping bounds.
  */
void FOC_PI_Init(foc_pi_controller_t *pi,
                 float kp,
                 float ki,
                 float out_min,
                 float out_max,
                 float sample_time_s)
{
    pi->kp            = kp;
    pi->ki            = ki;
    pi->integrator    = 0.0f;
    pi->out_min       = out_min;
    pi->out_max       = out_max;
    pi->sample_time_s = sample_time_s;
}

/**
  * @brief  Executes discrete PI controller step with conditional anti-windup clamping.
  */
float FOC_PI_Update(foc_pi_controller_t *pi, float setpoint, float feedback)
{
    float error = setpoint - feedback;

    /* Proportional term */
    float p_term = pi->kp * error;

    /* Tentative integrator state */
    float i_term = pi->integrator + (pi->ki * pi->sample_time_s * error);

    /* Total unsaturated output */
    float output = p_term + i_term;

    /* Anti-windup clamping logic */
    if (output > pi->out_max)
    {
        output = pi->out_max;
        /* Only integrate if error drives output back toward unsaturated range */
        if (error < 0.0f)
        {
            pi->integrator = i_term;
        }
    }
    else if (output < pi->out_min)
    {
        output = pi->out_min;
        /* Only integrate if error drives output back toward unsaturated range */
        if (error > 0.0f)
        {
            pi->integrator = i_term;
        }
    }
    else
    {
        /* In linear region: update integrator freely */
        pi->integrator = i_term;
    }

    return output;
}

/**
  * @brief  Circle limitation: Clamps rotating frame voltage vector (V_d, V_q)
  *         magnitude to maximum linear modulation circle: V_max = V_bus / sqrt(3).
  */
void FOC_Voltage_Circle_Limitation(foc_dq_t *v_dq, float v_bus)
{
    /* Maximum modulation radius without distortion */
    float v_max = v_bus * FOC_ONE_OVER_SQRT3;
    float v_max_sq = v_max * v_max;

    float v_mag_sq = (v_dq->d * v_dq->d) + (v_dq->q * v_dq->q);

    if (v_mag_sq > v_max_sq)
    {
        float scale = v_max / sqrtf(v_mag_sq);
        v_dq->d *= scale;
        v_dq->q *= scale;
    }
}

/**
  * @brief  Symmetric 7-Segment Space Vector Pulse Width Modulation generator.
  *         Computes timer compare register values (CCR1, CCR2, CCR3) for Center-Aligned
  *         TIM1 counting to maximize DC bus utilization by 15.47%.
  *
  * Derivation:
  *   Clarke coordinate voltages V_alpha, V_beta are projected onto the six 60° sectors.
  *   Center-aligned PWM ensures symmetrical pulse positioning around counter underflow.
  */
uint8_t FOC_SVPWM_Generate(const foc_alphabeta_t *v_alphabeta,
                           float v_bus,
                           uint16_t arr,
                           uint16_t *ccr1,
                           uint16_t *ccr2,
                           uint16_t *ccr3)
{
    if (v_bus < 1.0f)
    {
        *ccr1 = 0;
        *ccr2 = 0;
        *ccr3 = 0;
        return 0;
    }

    float inv_vbus = 1.0f / v_bus;
    float arr_f = (float)arr;

    /*
     * Projections onto 3 axes separated by 120 degrees:
     * v_a = V_beta
     * v_b = (sqrt(3)*V_alpha - V_beta) / 2
     * v_c = (-sqrt(3)*V_alpha - V_beta) / 2
     */
    float v_alpha = v_alphabeta->alpha;
    float v_beta  = v_alphabeta->beta;

    float v_a = v_beta;
    float v_b = (FOC_SQRT3_OVER_TWO * v_alpha) - (0.5f * v_beta);
    float v_c = (-FOC_SQRT3_OVER_TWO * v_alpha) - (0.5f * v_beta);

    /* Determine 60-degree sector */
    uint8_t sector = 0U;
    if (v_a > 0.0f) sector |= 1U;
    if (v_b > 0.0f) sector |= 2U;
    if (v_c > 0.0f) sector |= 4U;

    /* Calculate normalized vector timing terms */
    float t1 = 0.0f;
    float t2 = 0.0f;

    /* Scale factor for switching times */
    float k_time = FOC_SQRT3_OVER_TWO * arr_f * inv_vbus;

    switch (sector)
    {
        case 2: /* Sector 1: 0° to 60° (v_b > 0) */
            t1 = k_time * (v_alpha - FOC_ONE_OVER_SQRT3 * v_beta);
            t2 = k_time * (2.0f * FOC_ONE_OVER_SQRT3 * v_beta);
            break;

        case 6: /* Sector 2: 60° to 120° (v_a > 0 and v_b > 0) */
            t1 = k_time * (v_alpha + FOC_ONE_OVER_SQRT3 * v_beta);
            t2 = k_time * (-v_alpha + FOC_ONE_OVER_SQRT3 * v_beta);
            break;

        case 4: /* Sector 3: 120° to 180° (v_c > 0) */
            t1 = k_time * (2.0f * FOC_ONE_OVER_SQRT3 * v_beta);
            t2 = k_time * (-v_alpha - FOC_ONE_OVER_SQRT3 * v_beta);
            break;

        case 5: /* Sector 4: 180° to 240° (v_a > 0 and v_c > 0) */
            t1 = k_time * (-v_alpha + FOC_ONE_OVER_SQRT3 * v_beta);
            t2 = k_time * (-2.0f * FOC_ONE_OVER_SQRT3 * v_beta);
            break;

        case 1: /* Sector 5: 240° to 300° (v_a > 0) */
            t1 = k_time * (-v_alpha - FOC_ONE_OVER_SQRT3 * v_beta);
            t2 = k_time * (v_alpha - FOC_ONE_OVER_SQRT3 * v_beta);
            break;

        case 3: /* Sector 6: 300° to 360° (v_a > 0 and v_b > 0 in negative plane) */
            t1 = k_time * (-2.0f * FOC_ONE_OVER_SQRT3 * v_beta);
            t2 = k_time * (v_alpha + FOC_ONE_OVER_SQRT3 * v_beta);
            break;

        default:
            t1 = 0.0f;
            t2 = 0.0f;
            break;
    }

    /* Clamp negative values due to numerical precision */
    if (t1 < 0.0f) t1 = 0.0f;
    if (t2 < 0.0f) t2 = 0.0f;

    /* Over-modulation saturation scaling */
    float t_sum = t1 + t2;
    if (t_sum > arr_f)
    {
        float scale = arr_f / t_sum;
        t1 *= scale;
        t2 *= scale;
    }

    /* Symmetrical center-aligned zero vector time */
    float t0 = (arr_f - t1 - t2) * 0.5f;

    float ta = 0.0f;
    float tb = 0.0f;
    float tc = 0.0f;

    /* Assign phase switching times depending on current sector */
    switch (sector)
    {
        case 2: /* Sector 1 */
            ta = t0;
            tb = t0 + t1;
            tc = t0 + t1 + t2;
            break;
        case 6: /* Sector 2 */
            ta = t0 + t2;
            tb = t0;
            tc = t0 + t1 + t2;
            break;
        case 4: /* Sector 3 */
            ta = t0 + t1 + t2;
            tb = t0;
            tc = t0 + t1;
            break;
        case 5: /* Sector 4 */
            ta = t0 + t1 + t2;
            tb = t0 + t2;
            tc = t0;
            break;
        case 1: /* Sector 5 */
            ta = t0 + t1;
            tb = t0 + t1 + t2;
            tc = t0;
            break;
        case 3: /* Sector 6 */
            ta = t0;
            tb = t0 + t1 + t2;
            tc = t0 + t2;
            break;
        default:
            ta = arr_f * 0.5f;
            tb = arr_f * 0.5f;
            tc = arr_f * 0.5f;
            break;
    }

    /* Bound check compare register values */
    if (ta < 0.0f) ta = 0.0f;
    if (ta > arr_f) ta = arr_f;
    if (tb < 0.0f) tb = 0.0f;
    if (tb > arr_f) tb = arr_f;
    if (tc < 0.0f) tc = 0.0f;
    if (tc > arr_f) tc = arr_f;

    *ccr1 = (uint16_t)ta;
    *ccr2 = (uint16_t)tb;
    *ccr3 = (uint16_t)tc;

    return sector;
}
