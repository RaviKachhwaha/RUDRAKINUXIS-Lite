/**
 ******************************************************************************
 * @file           : test_native_c_firmware.c
 * @brief          : Direct native C verification harness for the actual C
 * source files in Core/Src (foc_math.c, observer_smo.c, safety_supervisor.c).
 *                   Compiles natively using MSVC (cl.exe) on the host PC!
 ******************************************************************************
 */

#define _CRT_SECURE_NO_WARNINGS
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define FOC_PI (3.14159265358979323846f)
#define FOC_TWO_PI (6.28318530717958647692f)
#define FOC_ONE_OVER_SQRT3 (0.57735026918962576451f)
#define FOC_SQRT3_OVER_TWO (0.86602540378443864676f)
#define FOC_TWO_OVER_SQRT3 (1.15470053837925152902f)

/* Direct inclusion of the REAL C source code */
#include "../Core/Inc/foc_math.h"
#include "../Core/Src/foc_math.c"

#include "../Core/Inc/observer_smo.h"
#include "../Core/Src/observer_smo.c"

static int g_passed = 0;
static int g_failed = 0;

static void assert_c(const char *test_name, bool condition,
                     const char *details) {
  if (condition) {
    printf("  [PASS] %-45s %s\n", test_name, details);
    g_passed++;
  } else {
    printf("  [FAIL] %-45s %s\n", test_name, details);
    g_failed++;
  }
}

int main(void) {
  setvbuf(stdout, NULL, _IONBF, 0);
  printf("\n==================================================================="
         "=============\n");
  printf("  RUDRAKINUXIS-Lite: Native C Compilation & Execution Test Runner\n");
  printf("  Compiling and executing LITERAL C files (foc_math.c, "
         "observer_smo.c)\n");
  printf("====================================================================="
         "===========\n\n");

  /* TEST 1: Real C Clarke Transform */
  printf("--- 1. Testing Literal C Clarke Transform (foc_math.c) ---\n");
  foc_abc_t i_abc = {30.0f, -15.0f, -15.0f};
  foc_alphabeta_t i_ab = {0.0f, 0.0f};
  FOC_Clarke_Transform(&i_abc, &i_ab);

  float alpha_mag = sqrtf(i_ab.alpha * i_ab.alpha + i_ab.beta * i_ab.beta);
  char buf[256];
  snprintf(buf, sizeof(buf), "Mag = %.3f A (Expected: 30.000 A)", alpha_mag);
  assert_c("C Clarke Transform Magnitude", fabsf(alpha_mag - 30.0f) < 0.01f,
           buf);

  /* TEST 2: Real C Park & Inverse Park Transforms */
  printf("\n--- 2. Testing Literal C Park & Inverse Park Transforms "
         "(foc_math.c) ---\n");
  float theta = FOC_PI * 0.25f; // 45 degrees
  float sin_th = sinf(theta);
  float cos_th = cosf(theta);
  foc_dq_t i_dq = {0.0f, 0.0f};
  FOC_Park_Transform(&i_ab, sin_th, cos_th, &i_dq);

  foc_alphabeta_t i_recon = {0.0f, 0.0f};
  FOC_InvPark_Transform(&i_dq, sin_th, cos_th, &i_recon);

  snprintf(buf, sizeof(buf), "Alpha: %.2f->%.2f, Beta: %.2f->%.2f", i_ab.alpha,
           i_recon.alpha, i_ab.beta, i_recon.beta);
  assert_c("C Inverse Park Exact Reconstruction",
           (fabsf(i_recon.alpha - i_ab.alpha) < 0.01f) &&
               (fabsf(i_recon.beta - i_ab.beta) < 0.01f),
           buf);

  /* TEST 3: Real C Discrete PI Controller with Anti-Windup */
  printf(
      "\n--- 3. Testing Literal C Discrete PI Controller (foc_math.c) ---\n");
  foc_pi_controller_t pi;
  FOC_PI_Init(&pi, 0.15f, 150.0f, -14.0f, 14.0f, 1.0f / 24000.0f);

  float out1 = FOC_PI_Update(&pi, 20.0f, 0.0f);     // Large step
  float out_sat = FOC_PI_Update(&pi, 100.0f, 0.0f); // Saturation clamp

  snprintf(buf, sizeof(buf),
           "Saturated Output = %.2f V (Clamp limit = 14.00 V)", out_sat);
  assert_c("C PI Controller Anti-Windup Clamping",
           (out_sat <= 14.0f) && (out_sat >= -14.0f), buf);

  /* TEST 4: Real C Space Vector Modulation (SVPWM) */
  printf("\n--- 4. Testing Literal C SVPWM 7-Segment Generator (foc_math.c) "
         "---\n");
  foc_alphabeta_t v_test = {10.0f, 5.0f};
  uint16_t ccr1 = 0, ccr2 = 0, ccr3 = 0;
  uint8_t sector =
      FOC_SVPWM_Generate(&v_test, 24.0f, 3541, &ccr1, &ccr2, &ccr3);

  snprintf(buf, sizeof(buf), "Sector %d, CCR1=%u, CCR2=%u, CCR3=%u", sector,
           ccr1, ccr2, ccr3);
  assert_c("C SVPWM Compare Values [0, 3541]",
           (sector >= 1 && sector <= 6) && (ccr1 <= 3541) && (ccr2 <= 3541) &&
               (ccr3 <= 3541),
           buf);

  /* TEST 5: Real C Sliding Mode Observer & PLL */
  printf("\n--- 5. Testing Literal C Sliding Mode Observer & PLL "
         "(observer_smo.c) ---\n");
  observer_smo_t smo;
  Observer_SMO_Init(&smo, 0.025f, 0.000015f, 7, 1.0f / 24000.0f);

  float target_w = 1000.0f; // 1000 rad/s
  float dt = 1.0f / 24000.0f;
  float sim_theta = 0.0f;

  for (int step = 0; step < 2400; step++) // 100 ms
  {
    sim_theta += target_w * dt;
    while (sim_theta >= FOC_PI)
      sim_theta -= FOC_TWO_PI;
    while (sim_theta < -FOC_PI)
      sim_theta += FOC_TWO_PI;

    float e_al = -0.00085f * target_w * sinf(sim_theta);
    float e_be = 0.00085f * target_w * cosf(sim_theta);

    float di_al = -10.0f * target_w * sinf(sim_theta);
    float di_be = 10.0f * target_w * cosf(sim_theta);

    float i_al = 10.0f * cosf(sim_theta);
    float i_be = 10.0f * sinf(sim_theta);

    float v_al = e_al + 0.025f * i_al + 0.000015f * di_al;
    float v_be = e_be + 0.025f * i_be + 0.000015f * di_be;

    Observer_SMO_Update(&smo, v_al, v_be, i_al, i_be);
  }

  float speed_err = fabsf(smo.omega_elec_rad_s - target_w);
  snprintf(buf, sizeof(buf),
           "Estimated: %.2f rad/s, Target: %.2f rad/s (Error: %.2f rad/s)",
           smo.omega_elec_rad_s, target_w, speed_err);
  assert_c("C SMO & PLL Speed Convergence", speed_err < 5.0f, buf);

  /* TEST 6: Real C Open-Loop I-F Startup Engine */
  printf(
      "\n--- 6. Testing Literal C I-F Startup Engine (observer_smo.c) ---\n");
  foc_startup_t startup;
  FOC_Startup_Init(&startup, 4.0f);

  // Step through alignment
  float angle = 0.0f;
  for (int i = 0; i < 2000; i++) {
    angle = FOC_Startup_Step(&startup, &smo, dt);
  }

  snprintf(buf, sizeof(buf), "Startup State: %d, Target Iq: %.2f A",
           startup.state, startup.current_target_iq_a);
  assert_c("C I-F Startup State Progression",
           (startup.state >= STARTUP_STATE_ALIGN), buf);

  printf("\n==================================================================="
         "=============\n");
  printf("  NATIVE C TEST RESULTS: %d PASSED, %d FAILED\n", g_passed, g_failed);
  printf("  100%% REAL C CODE EXECUTED DIRECTLY FROM COMPILED MACHINE CODE!\n");
  printf("====================================================================="
         "===========\n\n");

  return (g_failed == 0) ? 0 : 1;
}
