/**
  ******************************************************************************
  * @file           : foc_math.h
  * @brief          : High-speed mathematical engine for Field-Oriented Control.
  *                   Includes CORDIC hardware acceleration, Clarke/Park transforms,
  *                   SVPWM 7-segment duty generator, and discrete PI controllers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 RUDRAKINUXIS Project.
  * All rights reserved.
  *
  ******************************************************************************
  */

#ifndef __FOC_MATH_H
#define __FOC_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Mathematical Constants */
#define FOC_PI                          (3.14159265358979323846f)
#define FOC_TWO_PI                      (6.28318530717958647692f)
#define FOC_ONE_OVER_SQRT3              (0.57735026918962576451f) /* 1 / sqrt(3) */
#define FOC_SQRT3_OVER_TWO              (0.86602540378443864676f) /* sqrt(3) / 2 */
#define FOC_TWO_OVER_SQRT3              (1.15470053837925152902f) /* 2 / sqrt(3) */

/* Type Definitions */
typedef struct
{
    float a;
    float b;
    float c;
} foc_abc_t;

typedef struct
{
    float alpha;
    float beta;
} foc_alphabeta_t;

typedef struct
{
    float d;
    float q;
} foc_dq_t;

typedef struct
{
    float kp;               /**< Proportional gain */
    float ki;               /**< Integral gain */
    float integrator;       /**< Accumulated integrator state */
    float out_max;          /**< Maximum controller output limit */
    float out_min;          /**< Minimum controller output limit */
    float sample_time_s;    /**< Sampling period in seconds */
} foc_pi_controller_t;

/*============================================================================*/
/*                      FAST TRIGONOMETRIC FUNCTIONS                          */
/*============================================================================*/

/**
  * @brief  Computes Sine and Cosine simultaneously using STM32G4 Hardware CORDIC.
  *         Execution takes 6 clock cycles (~35 nanoseconds @ 170 MHz).
  * @param  angle_rad Angle in radians [-PI, +PI]
  * @param  sin_out   Pointer to store calculated sine value [-1.0, +1.0]
  * @param  cos_out   Pointer to store calculated cosine value [-1.0, +1.0]
  * @retval None
  */
void FOC_Cordic_SinCos(float angle_rad, float *sin_out, float *cos_out);

/**
  * @brief  Fast rational approximation of four-quadrant arctangent (atan2).
  *         Max error < 0.005 radians, 5x faster than standard libm atan2f.
  * @param  y Numerator
  * @param  x Denominator
  * @retval Angle in radians [-PI, +PI]
  */
float FOC_Fast_Atan2(float y, float x);

/**
  * @brief  Fast modulo normalization of angle to [-PI, +PI).
  * @param  angle_rad Input angle in radians
  * @retval Normalized angle in radians
  */
static inline float FOC_Angle_Normalize(float angle_rad)
{
    while (angle_rad >= FOC_PI)
    {
        angle_rad -= FOC_TWO_PI;
    }
    while (angle_rad < -FOC_PI)
    {
        angle_rad += FOC_TWO_PI;
    }
    return angle_rad;
}

/*============================================================================*/
/*                      VECTOR TRANSFORMATION ENGINE                          */
/*============================================================================*/

/**
  * @brief  Clarke Transformation: Converts 3-phase currents (Ia, Ib, Ic)
  *         into stationary orthogonal 2-phase frame (I_alpha, I_beta).
  *         Preserves current amplitude (Amplitude-Invariant formulation).
  *         Formula:
  *           I_alpha = Ia
  *           I_beta  = (Ia + 2*Ib) / sqrt(3) = (Ib - Ic) / sqrt(3)
  * @param  i_abc       Measured 3-phase currents in Amperes
  * @param  i_alphabeta Output stationary frame currents in Amperes
  * @retval None
  */
static inline void FOC_Clarke_Transform(const foc_abc_t *i_abc, foc_alphabeta_t *i_alphabeta)
{
    i_alphabeta->alpha = i_abc->a;
    i_alphabeta->beta  = (i_abc->a + 2.0f * i_abc->b) * FOC_ONE_OVER_SQRT3;
}

/**
  * @brief  Park Transformation: Rotates stationary orthogonal currents (I_alpha, I_beta)
  *         into synchronous rotating frame (I_d, I_q) aligned with rotor flux vector.
  *         Formula:
  *           I_d =  I_alpha * cos(theta) + I_beta * sin(theta)
  *           I_q = -I_alpha * sin(theta) + I_beta * cos(theta)
  * @param  i_alphabeta Stationary frame currents
  * @param  sin_theta   Precomputed sin(electrical_angle)
  * @param  cos_theta   Precomputed cos(electrical_angle)
  * @param  i_dq        Output rotating frame currents (d: flux, q: torque)
  * @retval None
  */
static inline void FOC_Park_Transform(const foc_alphabeta_t *i_alphabeta,
                                      float sin_theta,
                                      float cos_theta,
                                      foc_dq_t *i_dq)
{
    i_dq->d =  i_alphabeta->alpha * cos_theta + i_alphabeta->beta * sin_theta;
    i_dq->q = -i_alphabeta->alpha * sin_theta + i_alphabeta->beta * cos_theta;
}

/**
  * @brief  Inverse Park Transformation: Rotates rotating frame voltages (V_d, V_q)
  *         back into stationary orthogonal frame (V_alpha, V_beta).
  *         Formula:
  *           V_alpha = V_d * cos(theta) - V_q * sin(theta)
  *           V_beta  = V_d * sin(theta) + V_q * cos(theta)
  * @param  v_dq        Rotating frame voltages
  * @param  sin_theta   Precomputed sin(electrical_angle)
  * @param  cos_theta   Precomputed cos(electrical_angle)
  * @param  v_alphabeta Output stationary frame voltages
  * @retval None
  */
static inline void FOC_InvPark_Transform(const foc_dq_t *v_dq,
                                         float sin_theta,
                                         float cos_theta,
                                         foc_alphabeta_t *v_alphabeta)
{
    v_alphabeta->alpha = v_dq->d * cos_theta - v_dq->q * sin_theta;
    v_alphabeta->beta  = v_dq->d * sin_theta + v_dq->q * cos_theta;
}

/*============================================================================*/
/*                      PI CONTROLLER & VOLTAGE CLAMPING                      */
/*============================================================================*/

/**
  * @brief  Initializes a discrete PI controller with gains and clamping bounds.
  * @param  pi             Pointer to controller struct
  * @param  kp             Proportional gain
  * @param  ki             Integral gain
  * @param  out_min        Lower output clamp limit
  * @param  out_max        Upper output clamp limit
  * @param  sample_time_s  Sampling period in seconds
  * @retval None
  */
void FOC_PI_Init(foc_pi_controller_t *pi,
                 float kp,
                 float ki,
                 float out_min,
                 float out_max,
                 float sample_time_s);

/**
  * @brief  Resets the integrator state of the PI controller to zero.
  * @param  pi Pointer to controller struct
  * @retval None
  */
static inline void FOC_PI_Reset(foc_pi_controller_t *pi)
{
    pi->integrator = 0.0f;
}

/**
  * @brief  Executes discrete PI controller step with anti-windup clamping.
  * @param  pi        Pointer to controller struct
  * @param  setpoint  Desired target value
  * @param  feedback  Measured feedback value
  * @retval Saturated controller output
  */
float FOC_PI_Update(foc_pi_controller_t *pi, float setpoint, float feedback);

/**
  * @brief  Circle limitation: Clamps rotating frame voltage vector (V_d, V_q)
  *         magnitude to maximum linear modulation circle: V_max = V_bus / sqrt(3).
  * @param  v_dq   Pointer to voltage command struct (modified in place)
  * @param  v_bus  Measured DC bus voltage in Volts
  * @retval None
  */
void FOC_Voltage_Circle_Limitation(foc_dq_t *v_dq, float v_bus);

/*============================================================================*/
/*                      SPACE VECTOR MODULATION (SVPWM)                       */
/*============================================================================*/

/**
  * @brief  Symmetric 7-Segment Space Vector Pulse Width Modulation generator.
  *         Computes timer compare register values (CCR1, CCR2, CCR3) for Center-Aligned
  *         TIM1 counting to maximize DC bus utilization by 15.47%.
  * @param  v_alphabeta Stationary frame voltage commands in Volts
  * @param  v_bus       Current DC bus voltage in Volts
  * @param  arr         Timer Auto-Reload Register value (PWM_TIMER_ARR_VALUE)
  * @param  ccr1        Output pointer for Phase U compare register
  * @param  ccr2        Output pointer for Phase V compare register
  * @param  ccr3        Output pointer for Phase W compare register
  * @retval Sector index (1 to 6)
  */
uint8_t FOC_SVPWM_Generate(const foc_alphabeta_t *v_alphabeta,
                           float v_bus,
                           uint16_t arr,
                           uint16_t *ccr1,
                           uint16_t *ccr2,
                           uint16_t *ccr3);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_MATH_H */
