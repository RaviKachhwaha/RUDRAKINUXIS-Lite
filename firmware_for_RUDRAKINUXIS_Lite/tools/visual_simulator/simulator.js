/**
  ******************************************************************************
  * @file           : simulator.js
  * @brief          : Production-grade Web Visual Simulator for RUDRAKINUXIS-Lite
  *                   STM32G431 FOC ESC Firmware.
  ******************************************************************************
  */

// --- Firmware Constants Matching board_config.h ---
const FOC_FREQ_HZ = 24000;
const DT_SEC = 1.0 / FOC_FREQ_HZ;
const ARR_TICKS = 3541;
const RS_OHMS = 0.025;
const LS_HENRIES = 0.000015;
const POLE_PAIRS = 7;
const BEMF_CONST_WB = 0.00085;
const MAX_CONT_CURRENT_A = 27.0;
const MAX_PEAK_CURRENT_A = 45.0;
const SW_OCP_LIMIT_A = 55.0;
const HW_OCP_LIMIT_A = 65.0;
const VBUS_OVP_V = 26.5;
const VBUS_UVLO_V = 18.0;

// --- State Machine ---
const FOC_STATE = {
  CALIBRATION: 'CALIBRATION',
  DISARMED: 'DISARMED',
  ARMED: 'ARMED',
  ALIGN: 'ALIGN',
  STARTUP_RAMP: 'STARTUP_RAMP',
  CLOSED_LOOP: 'CLOSED_LOOP',
  FAULT: 'FAULT'
};

// --- Fault Flags ---
const FAULT_FLAGS = {
  NONE: 0,
  SW_OCP: 1 << 0,
  HW_OCP: 1 << 1,
  OVP: 1 << 2,
  UVLO: 1 << 3,
  OTP: 1 << 4,
  ROTOR_LOCK: 1 << 5,
  SIGNAL_TIMEOUT: 1 << 6
};

// --- Simulation Master State ---
const sim = {
  state: FOC_STATE.DISARMED,
  is_armed: false,
  active_faults: FAULT_FLAGS.NONE,
  
  // Controls
  throttle_raw: 0,
  throttle_norm: 0.0,
  v_bus: 24.0,
  load_current_override: 0.0,
  temp_c: 25.0,
  rotor_locked: false,
  signal_cut: false,

  // Motor Physical States
  theta_mech: 0.0,
  theta_elec: 0.0,
  omega_mech: 0.0,
  omega_elec: 0.0,
  rpm_mech: 0.0,
  rpm_elec: 0.0,

  // Current Vectors
  i_a: 0.0,
  i_b: 0.0,
  i_c: 0.0,
  i_alpha: 0.0,
  i_beta: 0.0,
  i_d: 0.0,
  i_q: 0.0,
  target_id: 0.0,
  target_iq: 0.0,
  max_iq_allowed: MAX_PEAK_CURRENT_A,

  // Voltage Commands & SVPWM
  v_d: 0.0,
  v_q: 0.0,
  v_alpha: 0.0,
  v_beta: 0.0,
  ccr1: 0,
  ccr2: 0,
  ccr3: 0,
  svpwm_sector: 1,

  // Sliding Mode Observer
  smo: {
    k_smo: 10.0,
    g_coeff: DT_SEC / LS_HENRIES,
    f_coeff: 1.0 - (RS_OHMS * DT_SEC / LS_HENRIES),
    epsilon: 10.0 * (DT_SEC / LS_HENRIES),
    cutoff_rad_s: 2.0 * Math.PI * 1200.0,
    lpf_alpha: (2.0 * Math.PI * 1200.0 * DT_SEC) / (1.0 + (2.0 * Math.PI * 1200.0 * DT_SEC)),
    i_al_est: 0.0,
    i_be_est: 0.0,
    e_al_est: 0.0,
    e_be_est: 0.0,
    bemf_mag: 0.0,
    theta_pll: 0.0,
    omega_pll: 0.0,
    pll_integrator: 0.0,
    pll_kp: 500.0,
    pll_ki: 50000.0,
    theta_comp: 0.0
  },

  // Open-Loop I-F Startup
  startup: {
    stage: 'IDLE',
    timer_s: 0.0,
    ol_speed: 0.0,
    ol_angle: 0.0,
    target_current: 4.0,
    current_iq: 0.0,
    blend_ratio: 0.0,
    is_closed_loop: false
  },

  // PI Integrators
  pi_d_int: 0.0,
  pi_q_int: 0.0,

  // Counters
  ocp_counter_ms: 0,
  ovp_counter_ms: 0,
  uvlo_counter_ms: 0,
  stall_counter_ms: 0,
  watchdog_counter_ms: 0,

  // History buffers for Oscilloscope
  scope_channel: 'phase',
  history_len: 200,
  history: {
    t: [],
    i_a: [], i_b: [], i_c: [],
    i_d: [], i_q: [],
    i_al: [], i_be: [],
    ccr1: [], ccr2: [], ccr3: [],
    e_al: [], e_be: []
  }
};

// --- Mathematical Helper Functions ---
function normalizeAngle(rad) {
  while (rad >= Math.PI) rad -= 2.0 * Math.PI;
  while (rad < -Math.PI) rad += 2.0 * Math.PI;
  return rad;
}

function clarkeTransform(ia, ib) {
  return {
    alpha: ia,
    beta: (ia + 2.0 * ib) * (1.0 / Math.sqrt(3.0))
  };
}

function parkTransform(al, be, sinTh, cosTh) {
  return {
    d:  al * cosTh + be * sinTh,
    q: -al * sinTh + be * cosTh
  };
}

function invParkTransform(d, q, sinTh, cosTh) {
  return {
    alpha: d * cosTh - q * sinTh,
    beta:  d * sinTh + q * cosTh
  };
}

function svpwmGenerate(v_alpha, v_beta, v_bus, arr) {
  if (v_bus < 1.0) return { sector: 1, ccr1: 0, ccr2: 0, ccr3: 0 };
  const arr_f = arr;
  const inv_vbus = 1.0 / v_bus;
  const sqrt3_over_2 = Math.sqrt(3.0) / 2.0;
  const one_over_sqrt3 = 1.0 / Math.sqrt(3.0);

  const va = v_beta;
  const vb = (sqrt3_over_2 * v_alpha) - (0.5 * v_beta);
  const vc = (-sqrt3_over_2 * v_alpha) - (0.5 * v_beta);

  let sector = 0;
  if (va > 0) sector |= 1;
  if (vb > 0) sector |= 2;
  if (vc > 0) sector |= 4;

  const k_time = sqrt3_over_2 * arr_f * inv_vbus;
  let t1 = 0, t2 = 0;

  switch (sector) {
    case 2:
      t1 = k_time * (v_alpha - one_over_sqrt3 * v_beta);
      t2 = k_time * (2.0 * one_over_sqrt3 * v_beta);
      break;
    case 6:
      t1 = k_time * (v_alpha + one_over_sqrt3 * v_beta);
      t2 = k_time * (-v_alpha + one_over_sqrt3 * v_beta);
      break;
    case 4:
      t1 = k_time * (2.0 * one_over_sqrt3 * v_beta);
      t2 = k_time * (-v_alpha - one_over_sqrt3 * v_beta);
      break;
    case 5:
      t1 = k_time * (-v_alpha + one_over_sqrt3 * v_beta);
      t2 = k_time * (-2.0 * one_over_sqrt3 * v_beta);
      break;
    case 1:
      t1 = k_time * (-v_alpha - one_over_sqrt3 * v_beta);
      t2 = k_time * (v_alpha - one_over_sqrt3 * v_beta);
      break;
    case 3:
      t1 = k_time * (-2.0 * one_over_sqrt3 * v_beta);
      t2 = k_time * (v_alpha + one_over_sqrt3 * v_beta);
      break;
    default:
      t1 = 0; t2 = 0;
  }

  t1 = Math.max(0, t1);
  t2 = Math.max(0, t2);
  const t_sum = t1 + t2;
  if (t_sum > arr_f) {
    const scale = arr_f / t_sum;
    t1 *= scale;
    t2 *= scale;
  }

  const t0 = (arr_f - t1 - t2) * 0.5;
  let ta, tb, tc;

  switch (sector) {
    case 2: ta = t0; tb = t0 + t1; tc = t0 + t1 + t2; break;
    case 6: ta = t0 + t2; tb = t0; tc = t0 + t1 + t2; break;
    case 4: ta = t0 + t1 + t2; tb = t0; tc = t0 + t1; break;
    case 5: ta = t0 + t1 + t2; tb = t0 + t2; tc = t0; break;
    case 1: ta = t0 + t1; tb = t0 + t1 + t2; tc = t0; break;
    case 3: ta = t0; tb = t0 + t1 + t2; tc = t0 + t2; break;
    default: ta = tb = tc = arr_f * 0.5;
  }

  return {
    sector: sector || 1,
    ccr1: Math.min(arr, Math.max(0, Math.round(ta))),
    ccr2: Math.min(arr, Math.max(0, Math.round(tb))),
    ccr3: Math.min(arr, Math.max(0, Math.round(tc)))
  };
}

// --- DShot CRC Calculation ---
function dshotCalcCRC(frame, inverted) {
  const cVal = frame >> 4;
  let crc = (cVal ^ (cVal >> 4) ^ (cVal >> 8)) & 0x0F;
  if (inverted) crc = (~crc) & 0x0F;
  return crc;
}

// --- FOC Discrete Step Execution ---
function executeFOCStep() {
  // If faulted or disarmed
  if (sim.state === FOC_STATE.FAULT || sim.state === FOC_STATE.DISARMED) {
    sim.i_a = 0; sim.i_b = 0; sim.i_c = 0;
    sim.i_d = 0; sim.i_q = 0;
    sim.v_d = 0; sim.v_q = 0;
    sim.ccr1 = 0; sim.ccr2 = 0; sim.ccr3 = 0;
    sim.omega_mech = Math.max(0, sim.omega_mech * 0.98);
    sim.omega_elec = sim.omega_mech * POLE_PAIRS;
    sim.theta_mech = normalizeAngle(sim.theta_mech + sim.omega_mech * DT_SEC * 10);
    sim.theta_elec = normalizeAngle(sim.theta_mech * POLE_PAIRS);
    return;
  }

  // Target current command
  const commanded_iq = sim.throttle_norm * sim.max_iq_allowed;

  // Startup State Machine
  if (sim.state === FOC_STATE.ARMED) {
    sim.state = FOC_STATE.ALIGN;
    sim.startup.stage = 'ALIGN';
    sim.startup.timer_s = 0.0;
    sim.startup.current_iq = 0.0;
    sim.startup.is_closed_loop = false;
  }

  if (sim.startup.stage === 'ALIGN') {
    sim.startup.timer_s += DT_SEC * 10;
    sim.startup.current_iq = Math.min(sim.startup.target_current, sim.startup.current_iq + 0.1);
    sim.target_iq = sim.startup.current_iq;
    sim.theta_elec = 0.0;

    if (sim.startup.timer_s >= 0.08) {
      sim.startup.stage = 'RAMP';
      sim.state = FOC_STATE.STARTUP_RAMP;
    }
  } else if (sim.startup.stage === 'RAMP') {
    sim.startup.ol_speed += 800.0 * (DT_SEC * 10);
    sim.startup.ol_angle = normalizeAngle(sim.startup.ol_angle + sim.startup.ol_speed * (DT_SEC * 10));
    sim.target_iq = sim.startup.target_current;
    sim.theta_elec = sim.startup.ol_angle;

    // Transition to blend if speed > 150 rad/s
    if (sim.startup.ol_speed >= 150.0 && sim.smo.bemf_mag >= 0.5) {
      sim.startup.stage = 'BLEND';
      sim.startup.blend_ratio = 0.0;
    }
  } else if (sim.startup.stage === 'BLEND') {
    sim.startup.blend_ratio += (DT_SEC * 10) / 0.04;
    if (sim.startup.blend_ratio >= 1.0) {
      sim.startup.stage = 'CLOSED_LOOP';
      sim.startup.is_closed_loop = true;
      sim.state = FOC_STATE.CLOSED_LOOP;
    }
    sim.target_iq = commanded_iq;
    sim.theta_elec = normalizeAngle(sim.startup.ol_angle * (1 - sim.startup.blend_ratio) + sim.smo.theta_comp * sim.startup.blend_ratio);
  } else {
    // Closed-loop FOC
    sim.target_iq = commanded_iq;
    sim.theta_elec = sim.smo.theta_comp;
    sim.state = FOC_STATE.CLOSED_LOOP;
  }

  // Motor Physics Integration
  if (!sim.rotor_locked) {
    const torque_elec = 1.5 * POLE_PAIRS * BEMF_CONST_WB * sim.i_q;
    const friction = 0.0005 * sim.omega_mech;
    const load_torque = 0.000002 * Math.pow(sim.omega_mech, 2) + (sim.load_current_override * 0.005);
    const alpha_mech = (torque_elec - friction - load_torque) / 0.00008; // Inertia J

    sim.omega_mech = Math.max(0, sim.omega_mech + alpha_mech * (DT_SEC * 10));
    sim.omega_elec = sim.omega_mech * POLE_PAIRS;
    sim.theta_mech = normalizeAngle(sim.theta_mech + sim.omega_mech * (DT_SEC * 10));
    if (sim.startup.is_closed_loop) {
      sim.theta_elec = normalizeAngle(sim.theta_mech * POLE_PAIRS);
    }
  } else {
    sim.omega_mech = 0;
    sim.omega_elec = 0;
  }

  sim.rpm_mech = sim.omega_mech * (60.0 / (2.0 * Math.PI));
  sim.rpm_elec = sim.omega_elec * (60.0 / (2.0 * Math.PI));

  // Current regulation (PI Controller on Id, Iq)
  const sinTh = Math.sin(sim.theta_elec);
  const cosTh = Math.cos(sim.theta_elec);

  // Cross-coupling decoupling feedforward
  const v_d_ff = -sim.omega_elec * LS_HENRIES * sim.i_q;
  const v_q_ff =  sim.omega_elec * (LS_HENRIES * sim.i_d + BEMF_CONST_WB);

  // PI Updates
  const kp = 0.15, ki = 150.0;
  const err_d = sim.target_id - sim.i_d;
  const err_q = sim.target_iq - sim.i_q;

  sim.pi_d_int = Math.max(-14, Math.min(14, sim.pi_d_int + ki * DT_SEC * err_d));
  sim.pi_q_int = Math.max(-14, Math.min(14, sim.pi_q_int + ki * DT_SEC * err_q));

  sim.v_d = (kp * err_d + sim.pi_d_int) + v_d_ff;
  sim.v_q = (kp * err_q + sim.pi_q_int) + v_q_ff;

  // Circle limitation
  const v_max = sim.v_bus / Math.sqrt(3.0);
  const v_mag = Math.sqrt(sim.v_d * sim.v_d + sim.v_q * sim.v_q);
  if (v_mag > v_max) {
    const scale = v_max / v_mag;
    sim.v_d *= scale;
    sim.v_q *= scale;
  }

  // Current response dynamics
  sim.i_q += (sim.target_iq - sim.i_q) * 0.12;
  sim.i_d += (sim.target_id - sim.i_d) * 0.12;

  // Add synthetic load current override if user requested stress test
  if (sim.load_current_override > 0) {
    sim.i_q = sim.load_current_override;
  }

  // Inverse Park -> Clarke
  const v_ab = invParkTransform(sim.v_d, sim.v_q, sinTh, cosTh);
  sim.v_alpha = v_ab.alpha;
  sim.v_beta = v_ab.beta;

  const i_ab = invParkTransform(sim.i_d, sim.i_q, sinTh, cosTh);
  sim.i_alpha = i_ab.alpha;
  sim.i_beta = i_ab.beta;

  // Phase Currents Reconstruction
  sim.i_a = sim.i_alpha;
  sim.i_b = -0.5 * sim.i_alpha + (Math.sqrt(3.0) / 2.0) * sim.i_beta;
  sim.i_c = -(sim.i_a + sim.i_b);

  // SVPWM Duty generation
  const svpwm = svpwmGenerate(sim.v_alpha, sim.v_beta, sim.v_bus, ARR_TICKS);
  sim.svpwm_sector = svpwm.sector;
  sim.ccr1 = svpwm.ccr1;
  sim.ccr2 = svpwm.ccr2;
  sim.ccr3 = svpwm.ccr3;

  // Update Sliding Mode Observer
  updateSMO();
}

function updateSMO() {
  const smo = sim.smo;
  const err_al = smo.i_al_est - sim.i_alpha;
  const err_be = smo.i_be_est - sim.i_beta;

  const sat_al = Math.max(-1.0, Math.min(1.0, err_al / smo.epsilon));
  const sat_be = Math.max(-1.0, Math.min(1.0, err_be / smo.epsilon));
  const z_al = smo.k_smo * sat_al;
  const z_be = smo.k_smo * sat_be;

  smo.i_al_est = (smo.f_coeff * smo.i_al_est) + (smo.g_coeff * (sim.v_alpha - z_al));
  smo.i_be_est = (smo.f_coeff * smo.i_be_est) + (smo.g_coeff * (sim.v_beta  - z_be));

  smo.e_al_est += smo.lpf_alpha * (z_al - smo.e_al_est);
  smo.e_be_est += smo.lpf_alpha * (z_be - smo.e_be_est);
  smo.bemf_mag = Math.sqrt(smo.e_al_est * smo.e_al_est + smo.e_be_est * smo.e_be_est);

  const sin_pll = Math.sin(smo.theta_pll);
  const cos_pll = Math.cos(smo.theta_pll);
  let pll_err = (-smo.e_al_est * cos_pll) - (smo.e_be_est * sin_pll);
  if (smo.bemf_mag > 0.05) {
    pll_err /= smo.bemf_mag;
  } else {
    pll_err = 0.0;
  }

  smo.pll_integrator += smo.pll_ki * (DT_SEC * 10) * pll_err;
  smo.omega_pll = smo.pll_kp * pll_err + smo.pll_integrator;
  smo.theta_pll = normalizeAngle(smo.theta_pll + smo.omega_pll * (DT_SEC * 10));

  const phase_delay = Math.atan2(smo.omega_pll, smo.cutoff_rad_s);
  smo.theta_comp = normalizeAngle(smo.theta_pll + phase_delay);
}

// --- 1 kHz Safety Supervisor Task ---
function executeSafetyTask1kHz() {
  const current_mag = Math.sqrt(sim.i_alpha * sim.i_alpha + sim.i_beta * sim.i_beta);

  // 1. Hardware OCP (>65A instantaneous)
  if (current_mag > HW_OCP_LIMIT_A) {
    sim.active_faults |= FAULT_FLAGS.HW_OCP;
    tripFault("HARDWARE COMP1 OCP (<50ns trip @ " + current_mag.toFixed(1) + "A)");
  }

  // 2. Software OCP (>55A > 5ms)
  if (current_mag > SW_OCP_LIMIT_A) {
    sim.ocp_counter_ms += 10;
    if (sim.ocp_counter_ms >= 5) {
      sim.active_faults |= FAULT_FLAGS.SW_OCP;
      tripFault("SOFTWARE OCP (>55A persisted >5ms)");
    }
  } else {
    sim.ocp_counter_ms = Math.max(0, sim.ocp_counter_ms - 2);
  }

  // 3. OVP (>26.5V)
  if (sim.v_bus > VBUS_OVP_V) {
    sim.ovp_counter_ms += 10;
    if (sim.ovp_counter_ms >= 10) {
      sim.active_faults |= FAULT_FLAGS.OVP;
      tripFault("OVER-VOLTAGE PROTECTION (Vbus = " + sim.v_bus.toFixed(1) + "V > 26.5V)");
    }
  } else {
    sim.ovp_counter_ms = Math.max(0, sim.ovp_counter_ms - 2);
  }

  // 4. UVLO (<18.0V)
  if (sim.v_bus < VBUS_UVLO_V && sim.is_armed) {
    sim.uvlo_counter_ms += 10;
    if (sim.uvlo_counter_ms >= 20) {
      sim.active_faults |= FAULT_FLAGS.UVLO;
      tripFault("UNDER-VOLTAGE LOCKOUT (Vbus = " + sim.v_bus.toFixed(1) + "V < 18.0V)");
    }
  } else {
    sim.uvlo_counter_ms = Math.max(0, sim.uvlo_counter_ms - 2);
  }

  // 5. Thermal Supervisor (Derate > 85°C, Trip > 105°C)
  if (sim.temp_c >= 105.0) {
    sim.active_faults |= FAULT_FLAGS.OTP;
    tripFault("OVER-TEMPERATURE HARD CUTOFF (T = " + sim.temp_c.toFixed(1) + "°C > 105°C)");
  } else if (sim.temp_c >= 85.0) {
    const factor = (105.0 - sim.temp_c) / (105.0 - 85.0);
    sim.max_iq_allowed = 10.0 + factor * (MAX_PEAK_CURRENT_A - 10.0);
  } else {
    sim.max_iq_allowed = MAX_PEAK_CURRENT_A;
  }

  // 6. Rotor Stall Detection (>15A and speed < 15 rad/s > 250ms)
  if (sim.is_armed && Math.abs(sim.i_q) > 15.0 && Math.abs(sim.omega_elec) < 15.0) {
    sim.stall_counter_ms += 10;
    if (sim.stall_counter_ms >= 250) {
      sim.active_faults |= FAULT_FLAGS.ROTOR_LOCK;
      tripFault("ROTOR STALL DETECTED (High torque with locked rotor > 250ms)");
    }
  } else {
    sim.stall_counter_ms = 0;
  }

  // 7. Signal Watchdog (>100ms missing signal)
  if (sim.signal_cut && sim.is_armed) {
    sim.watchdog_counter_ms += 10;
    if (sim.watchdog_counter_ms >= 100) {
      sim.active_faults |= FAULT_FLAGS.SIGNAL_TIMEOUT;
      disarmESC();
      logCLI("SIGNAL WATCHDOG TIMEOUT: Throttle signal lost > 100ms. Disarmed.");
    }
  } else {
    sim.watchdog_counter_ms = 0;
  }
}

function tripFault(reason) {
  sim.state = FOC_STATE.FAULT;
  sim.is_armed = false;
  sim.i_a = 0; sim.i_b = 0; sim.i_c = 0;
  sim.ccr1 = 0; sim.ccr2 = 0; sim.ccr3 = 0;
  logCLI("!!! EMERGENCY SHUTDOWN: " + reason);
}

function armESC() {
  if (sim.active_faults !== FAULT_FLAGS.NONE) {
    logCLI("CANNOT ARM: Safety fault active! Clear faults first.");
    return false;
  }
  sim.is_armed = true;
  sim.state = FOC_STATE.ARMED;
  logCLI("ESC ARMED: Inverter PWM active.");
  return true;
}

function disarmESC() {
  sim.is_armed = false;
  if (sim.state !== FOC_STATE.FAULT) {
    sim.state = FOC_STATE.DISARMED;
  }
  sim.ccr1 = 0; sim.ccr2 = 0; sim.ccr3 = 0;
  logCLI("ESC DISARMED: Gate outputs forced Low.");
}

function clearFaults() {
  if (sim.v_bus >= VBUS_UVLO_V && sim.v_bus <= VBUS_OVP_V && sim.temp_c < 85.0 && !sim.rotor_locked) {
    sim.active_faults = FAULT_FLAGS.NONE;
    sim.load_current_override = 0;
    document.getElementById('slider-load').value = 0;
    document.getElementById('disp-load').textContent = '0.0 A';
    if (sim.state === FOC_STATE.FAULT) {
      sim.state = FOC_STATE.DISARMED;
    }
    logCLI("Faults successfully cleared. System ready.");
    return true;
  } else {
    logCLI("CANNOT CLEAR FAULTS: Operating parameters still violate safe limits.");
    return false;
  }
}

// --- Oscilloscope Drawing ---
function drawOscilloscope() {
  const canvas = document.getElementById('scope-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const w = canvas.width;
  const h = canvas.height;

  // Background
  ctx.fillStyle = '#080c14';
  ctx.fillRect(0, 0, w, h);

  // Grid
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
  ctx.lineWidth = 1;
  for (let x = 0; x < w; x += 40) {
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
  }
  for (let y = 0; y < h; y += 30) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }

  // Zero Center Line
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
  ctx.beginPath(); ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2); ctx.stroke();

  // Helper to draw trace
  function plotTrace(dataArray, color, scale, yOffset) {
    if (dataArray.length < 2) return;
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 0; i < dataArray.length; i++) {
      const x = (i / (sim.history_len - 1)) * w;
      const y = (h / 2 + yOffset) - (dataArray[i] * scale);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }

  const legend = document.getElementById('scope-legend');

  if (sim.scope_channel === 'phase') {
    plotTrace(sim.history.i_a, '#ef4444', 2.0, 0);
    plotTrace(sim.history.i_b, '#10b981', 2.0, 0);
    plotTrace(sim.history.i_c, '#38bdf8', 2.0, 0);
    legend.innerHTML = `
      <div class="legend-item"><span class="legend-color" style="background:#ef4444"></span> Phase U (${sim.i_a.toFixed(1)}A)</div>
      <div class="legend-item"><span class="legend-color" style="background:#10b981"></span> Phase V (${sim.i_b.toFixed(1)}A)</div>
      <div class="legend-item"><span class="legend-color" style="background:#38bdf8"></span> Phase W (${sim.i_c.toFixed(1)}A)</div>
    `;
  } else if (sim.scope_channel === 'park') {
    plotTrace(sim.history.i_d, '#f59e0b', 2.5, 0);
    plotTrace(sim.history.i_q, '#00f2fe', 2.5, 0);
    legend.innerHTML = `
      <div class="legend-item"><span class="legend-color" style="background:#f59e0b"></span> Flux Id (${sim.i_d.toFixed(2)}A -> 0.0A)</div>
      <div class="legend-item"><span class="legend-color" style="background:#00f2fe"></span> Torque Iq (${sim.i_q.toFixed(2)}A / Target ${sim.target_iq.toFixed(1)}A)</div>
    `;
  } else if (sim.scope_channel === 'clarke') {
    plotTrace(sim.history.i_al, '#8b5cf6', 2.0, 0);
    plotTrace(sim.history.i_be, '#ec4899', 2.0, 0);
    legend.innerHTML = `
      <div class="legend-item"><span class="legend-color" style="background:#8b5cf6"></span> I_alpha (${sim.i_alpha.toFixed(1)}A)</div>
      <div class="legend-item"><span class="legend-color" style="background:#ec4899"></span> I_beta (${sim.i_beta.toFixed(1)}A)</div>
    `;
  } else if (sim.scope_channel === 'svpwm') {
    plotTrace(sim.history.ccr1, '#38bdf8', 0.05, 50);
    plotTrace(sim.history.ccr2, '#10b981', 0.05, 50);
    plotTrace(sim.history.ccr3, '#f59e0b', 0.05, 50);
    legend.innerHTML = `
      <div class="legend-item"><span class="legend-color" style="background:#38bdf8"></span> TIM1 CCR1 (${sim.ccr1})</div>
      <div class="legend-item"><span class="legend-color" style="background:#10b981"></span> TIM1 CCR2 (${sim.ccr2})</div>
      <div class="legend-item"><span class="legend-color" style="background:#f59e0b"></span> TIM1 CCR3 (${sim.ccr3})</div>
    `;
  } else if (sim.scope_channel === 'smo') {
    plotTrace(sim.history.e_al, '#06b6d4', 30.0, 0);
    plotTrace(sim.history.e_be, '#eab308', 30.0, 0);
    legend.innerHTML = `
      <div class="legend-item"><span class="legend-color" style="background:#06b6d4"></span> SMO e_alpha (${sim.smo.e_al_est.toFixed(2)}V)</div>
      <div class="legend-item"><span class="legend-color" style="background:#eab308"></span> SMO e_beta (${sim.smo.e_be_est.toFixed(2)}V)</div>
    `;
  }
}

// --- Motor & Stator Dynamic Visualizer ---
function drawMotorCanvas() {
  const canvas = document.getElementById('motor-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const w = canvas.width;
  const h = canvas.height;
  const cx = w / 2;
  const cy = h / 2;
  const rStator = 140;
  const rRotor = 90;

  ctx.clearRect(0, 0, w, h);

  // Outer Stator Ring
  ctx.strokeStyle = '#1e293b';
  ctx.lineWidth = 14;
  ctx.beginPath();
  ctx.arc(cx, cy, rStator, 0, 2 * Math.PI);
  ctx.stroke();

  // 12 Stator Coils
  const numCoils = 12;
  for (let i = 0; i < numCoils; i++) {
    const angle = (i / numCoils) * 2 * Math.PI - Math.PI / 2;
    const x = cx + Math.cos(angle) * (rStator - 16);
    const y = cy + Math.sin(angle) * (rStator - 16);

    // Phase mapping: U, V, W repeating
    const phaseIdx = i % 3;
    let current = 0;
    let coilColor = '#475569';

    if (phaseIdx === 0) {
      current = sim.i_a;
      coilColor = current > 0 ? '#ef4444' : '#b91c1c';
    } else if (phaseIdx === 1) {
      current = sim.i_b;
      coilColor = current > 0 ? '#10b981' : '#047857';
    } else {
      current = sim.i_c;
      coilColor = current > 0 ? '#38bdf8' : '#0284c7';
    }

    const intensity = Math.min(1.0, Math.abs(current) / 30.0);

    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(angle + Math.PI / 2);

    ctx.fillStyle = coilColor;
    ctx.globalAlpha = 0.3 + 0.7 * intensity;
    ctx.shadowColor = coilColor;
    ctx.shadowBlur = intensity > 0.3 ? 12 : 0;
    ctx.fillRect(-10, -14, 20, 28);
    ctx.restore();
  }

  // Permanent Magnet Rotor (14 Poles = 7 Pole Pairs)
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(sim.theta_mech);

  // Rotor Core
  ctx.fillStyle = '#0f172a';
  ctx.beginPath();
  ctx.arc(0, 0, rRotor, 0, 2 * Math.PI);
  ctx.fill();
  ctx.strokeStyle = '#334155';
  ctx.lineWidth = 3;
  ctx.stroke();

  // 14 Rotor Magnets alternating N / S
  const numMagnets = 14;
  for (let m = 0; m < numMagnets; m++) {
    const mAngle = (m / numMagnets) * 2 * Math.PI;
    const mx = Math.cos(mAngle) * (rRotor - 10);
    const my = Math.sin(mAngle) * (rRotor - 10);
    const isNorth = (m % 2 === 0);

    ctx.save();
    ctx.translate(mx, my);
    ctx.rotate(mAngle + Math.PI / 2);
    ctx.fillStyle = isNorth ? '#ef4444' : '#38bdf8';
    ctx.fillRect(-6, -8, 12, 16);
    ctx.fillStyle = '#fff';
    ctx.font = '8px Inter';
    ctx.textAlign = 'center';
    ctx.fillText(isNorth ? 'N' : 'S', 0, 3);
    ctx.restore();
  }

  // Rotor Shaft
  ctx.fillStyle = '#1e293b';
  ctx.beginPath();
  ctx.arc(0, 0, 24, 0, 2 * Math.PI);
  ctx.fill();
  ctx.strokeStyle = '#64748b';
  ctx.lineWidth = 2;
  ctx.stroke();

  // Center d-axis pointer
  ctx.strokeStyle = '#00f2fe';
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(rRotor - 5, 0);
  ctx.stroke();

  ctx.restore();

  // Stator Magnetic Field Vector B_stator
  const bMag = Math.min(60, Math.sqrt(sim.i_alpha * sim.i_alpha + sim.i_beta * sim.i_beta) * 2);
  const bAngle = Math.atan2(sim.i_beta, sim.i_alpha);
  ctx.strokeStyle = '#f59e0b';
  ctx.lineWidth = 3;
  ctx.shadowColor = '#f59e0b';
  ctx.shadowBlur = 8;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx + Math.cos(bAngle) * bMag, cy + Math.sin(bAngle) * bMag);
  ctx.stroke();
  ctx.shadowBlur = 0;
}

// --- UI Update Loop ---
function updateUI() {
  // Update Header System Pills
  const valFocState = document.getElementById('val-foc-state');
  const pillFocState = document.getElementById('pill-foc-state');
  valFocState.textContent = sim.state;
  pillFocState.className = 'pill';
  if (sim.state === FOC_STATE.CLOSED_LOOP || sim.state === FOC_STATE.ARMED) {
    pillFocState.classList.add('state-armed');
  } else if (sim.state === FOC_STATE.FAULT) {
    pillFocState.classList.add('state-fault');
  }

  document.getElementById('val-loop-mode').textContent = sim.startup.is_closed_loop ? 'CLOSED LOOP (SMO)' : sim.startup.stage;

  // LEDs
  const led1 = document.getElementById('hw-led1');
  const led2 = document.getElementById('hw-led2');

  led1.className = 'led-bulb led-green ' + (sim.v_bus > 15.0 ? 'on' : '');
  led2.className = 'led-bulb led-blue';
  if (sim.active_faults !== FAULT_FLAGS.NONE || sim.state === FOC_STATE.FAULT) {
    led2.classList.add('strobe');
  } else if (sim.is_armed) {
    led2.classList.add('on');
  } else {
    led2.classList.add('heartbeat');
  }

  // Motor Overlay Stats
  document.getElementById('val-theta-elec').textContent = ((sim.theta_elec * 180 / Math.PI + 360) % 360).toFixed(1) + '°';
  document.getElementById('val-theta-mech').textContent = ((sim.theta_mech * 180 / Math.PI + 360) % 360).toFixed(1) + '°';
  document.getElementById('val-bemf-mag').textContent = sim.smo.bemf_mag.toFixed(2) + ' V';
  document.getElementById('val-svpwm-sector').textContent = 'Sector ' + sim.svpwm_sector;
  document.getElementById('badge-rpm').textContent = Math.round(sim.rpm_mech) + ' RPM';

  // Metrics Bar
  document.getElementById('val-metric-vbus').innerHTML = sim.v_bus.toFixed(2) + ' <span class="unit">V</span>';
  document.getElementById('bar-vbus').style.width = Math.min(100, (sim.v_bus / 28.0) * 100) + '%';

  const phaseRMS = Math.sqrt((sim.i_a * sim.i_a + sim.i_b * sim.i_b + sim.i_c * sim.i_c) / 3.0);
  document.getElementById('val-metric-curr').innerHTML = phaseRMS.toFixed(2) + ' <span class="unit">A</span>';
  document.getElementById('bar-curr').style.width = Math.min(100, (phaseRMS / 45.0) * 100) + '%';

  const powerW = sim.v_bus * Math.abs(sim.i_q);
  document.getElementById('val-metric-pwr').innerHTML = powerW.toFixed(1) + ' <span class="unit">W</span>';
  document.getElementById('bar-pwr').style.width = Math.min(100, (powerW / 600.0) * 100) + '%';

  document.getElementById('val-metric-temp').innerHTML = sim.temp_c.toFixed(1) + ' <span class="unit">°C</span>';
  document.getElementById('bar-temp').style.width = Math.min(100, (sim.temp_c / 120.0) * 100) + '%';

  // DShot Packet Monitor
  const dshotThrottleVal = sim.throttle_raw;
  const dshotFrame = (dshotThrottleVal << 5) | (0 << 4);
  const dshotCrc = dshotCalcCRC(dshotFrame, false);
  const fullPacket = dshotFrame | dshotCrc;

  document.getElementById('dshot-raw-hex').textContent = '0x' + fullPacket.toString(16).toUpperCase().padStart(4, '0') + 
    ' (0b' + fullPacket.toString(2).padStart(16, '0') + ')';
  document.getElementById('dshot-throttle-val').textContent = `${dshotThrottleVal} / 2047 (${(sim.throttle_norm * 100).toFixed(1)}%)`;
  document.getElementById('dshot-crc-val').textContent = `VALID (0x${dshotCrc.toString(16).toUpperCase()})`;

  const eRPM = Math.round(sim.rpm_elec);
  document.getElementById('dshot-erpm-hex').textContent = `0x${((eRPM & 0xFFFF)).toString(16).toUpperCase().padStart(4, '0')} (${eRPM} eRPM)`;

  // CAN FD Monitor
  const vbus_mv = Math.round(sim.v_bus * 1000);
  const curr_ca = Math.round(phaseRMS * 100);
  document.getElementById('can-tx-payload').textContent = 
    `${vbus_mv}mV | ${curr_ca}cA | ${Math.round(sim.rpm_mech)}RPM | ${Math.round(sim.temp_c)}C | STAT:0x${sim.active_faults.toString(16).padStart(2, '0')}`;
  document.getElementById('can-rx-payload').textContent = 
    `CMD: ${(sim.throttle_norm * 100).toFixed(1)}% | ARM: ${sim.is_armed ? 1 : 0}`;

  // Fault Tags
  const tagContainer = document.getElementById('fault-tags-container');
  tagContainer.innerHTML = '';
  if (sim.active_faults === FAULT_FLAGS.NONE) {
    tagContainer.innerHTML = '<span class="fault-tag ok">NO FAULTS ACTIVE</span>';
  } else {
    if (sim.active_faults & FAULT_FLAGS.SW_OCP) tagContainer.innerHTML += '<span class="fault-tag active">FAULT_SW_OCP (55A)</span>';
    if (sim.active_faults & FAULT_FLAGS.HW_OCP) tagContainer.innerHTML += '<span class="fault-tag active">FAULT_HW_OCP (65A COMP1)</span>';
    if (sim.active_faults & FAULT_FLAGS.OVP) tagContainer.innerHTML += '<span class="fault-tag active">FAULT_OVP (>26.5V)</span>';
    if (sim.active_faults & FAULT_FLAGS.UVLO) tagContainer.innerHTML += '<span class="fault-tag active">FAULT_UVLO (<18.0V)</span>';
    if (sim.active_faults & FAULT_FLAGS.OTP) tagContainer.innerHTML += '<span class="fault-tag active">FAULT_OTP (>105°C)</span>';
    if (sim.active_faults & FAULT_FLAGS.ROTOR_LOCK) tagContainer.innerHTML += '<span class="fault-tag active">FAULT_ROTOR_LOCK (250ms)</span>';
    if (sim.active_faults & FAULT_FLAGS.SIGNAL_TIMEOUT) tagContainer.innerHTML += '<span class="fault-tag active">FAULT_SIGNAL_TIMEOUT (100ms)</span>';
  }

  // Arm/Disarm Button Label
  const armBtn = document.getElementById('btn-arm-toggle');
  if (sim.is_armed) {
    armBtn.textContent = 'DISARM ESC';
    armBtn.className = 'btn btn-danger';
  } else {
    armBtn.textContent = 'ARM ESC';
    armBtn.className = 'btn btn-primary';
  }
}

// --- CLI Output Logger ---
function logCLI(text, isCmd = false) {
  const out = document.getElementById('cli-output');
  if (!out) return;
  const line = document.createElement('div');
  line.className = 'cli-line ' + (isCmd ? 'cmd' : 'resp');
  line.textContent = (isCmd ? '> ' : '') + text;
  out.appendChild(line);
  out.scrollTop = out.scrollHeight;
}

// --- Parse CLI Commands ---
function parseCLICommand(cmd) {
  const trimmed = cmd.trim().toUpperCase();
  logCLI(trimmed, true);

  if (trimmed === 'HELP') {
    logCLI('=== RUDRAKINUXIS-Lite Serial CLI Commands ===');
    logCLI('STATUS       : Print system telemetry and state');
    logCLI('ARM          : Arm ESC for motor rotation');
    logCLI('DISARM       : Disarm inverter gates');
    logCLI('SET_IQ <val> : Set target torque current in Amps');
    logCLI('CLEAR        : Clear active safety fault flags');
    logCLI('REBOOT_BL    : Jump to ST Factory ROM Bootloader');
    logCLI('============================================');
  } else if (trimmed === 'STATUS') {
    logCLI(`State: ${sim.state} | V_bus: ${sim.v_bus.toFixed(2)}V | Iq: ${sim.i_q.toFixed(2)}A | Id: ${sim.i_d.toFixed(2)}A`);
    logCLI(`RPM: ${Math.round(sim.rpm_mech)} | Temp: ${sim.temp_c.toFixed(1)}°C | Faults: 0x${sim.active_faults.toString(16)} | ISR: 5.09us`);
  } else if (trimmed === 'ARM') {
    armESC();
  } else if (trimmed === 'DISARM') {
    disarmESC();
  } else if (trimmed.startsWith('SET_IQ ')) {
    const val = parseFloat(trimmed.replace('SET_IQ ', ''));
    if (!isNaN(val)) {
      sim.target_iq = val;
      logCLI(`Target Iq set to ${val.toFixed(2)} A`);
    } else {
      logCLI("Invalid Iq value");
    }
  } else if (trimmed === 'CLEAR') {
    clearFaults();
  } else if (trimmed === 'REBOOT_BL') {
    triggerBootloaderJump();
  } else {
    logCLI(`Unknown command '${trimmed}'. Type 'HELP'.`);
  }
}

function triggerBootloaderJump() {
  logCLI("Writing 0xDEADBEEF to TAMP->BKP0R...");
  logCLI("Invoking NVIC_SystemReset()...");
  logCLI("Jumping to ST ROM Bootloader at 0x1FFF0000!");
  document.getElementById('modal-bootloader').classList.remove('hidden');
}

// --- Main Animation & Simulation Loop ---
let lastSafetyTime = performance.now();

function simulationLoop(timestamp) {
  // Run FOC steps
  executeFOCStep();

  // Run 1kHz Safety Supervisor every 1ms
  if (timestamp - lastSafetyTime >= 1.0) {
    executeSafetyTask1kHz();
    lastSafetyTime = timestamp;
  }

  // Push history for oscilloscope
  sim.history.t.push(timestamp);
  sim.history.i_a.push(sim.i_a);
  sim.history.i_b.push(sim.i_b);
  sim.history.i_c.push(sim.i_c);
  sim.history.i_d.push(sim.i_d);
  sim.history.i_q.push(sim.i_q);
  sim.history.i_al.push(sim.i_alpha);
  sim.history.i_be.push(sim.i_beta);
  sim.history.ccr1.push(sim.ccr1);
  sim.history.ccr2.push(sim.ccr2);
  sim.history.ccr3.push(sim.ccr3);
  sim.history.e_al.push(sim.smo.e_al_est);
  sim.history.e_be.push(sim.smo.e_be_est);

  if (sim.history.i_a.length > sim.history_len) {
    sim.history.t.shift();
    sim.history.i_a.shift(); sim.history.i_b.shift(); sim.history.i_c.shift();
    sim.history.i_d.shift(); sim.history.i_q.shift();
    sim.history.i_al.shift(); sim.history.i_be.shift();
    sim.history.ccr1.shift(); sim.history.ccr2.shift(); sim.history.ccr3.shift();
    sim.history.e_al.shift(); sim.history.e_be.shift();
  }

  // Draw visualizations
  drawMotorCanvas();
  drawOscilloscope();
  updateUI();

  requestAnimationFrame(simulationLoop);
}

// --- DOM Event Listeners & Wiring ---
window.addEventListener('DOMContentLoaded', () => {
  // Throttle Slider
  const sliderThrottle = document.getElementById('slider-throttle');
  const dispThrottle = document.getElementById('disp-throttle');
  sliderThrottle.addEventListener('input', (e) => {
    const raw = parseInt(e.target.value);
    sim.throttle_raw = raw;
    if (raw < 48) {
      sim.throttle_norm = 0.0;
    } else {
      sim.throttle_norm = (raw - 48) / (2047 - 48);
    }
    dispThrottle.textContent = `${(sim.throttle_norm * 100).toFixed(1)}% (${raw})`;
  });

  // Bus Voltage Slider
  const sliderVbus = document.getElementById('slider-vbus');
  const dispVbus = document.getElementById('disp-vbus');
  sliderVbus.addEventListener('input', (e) => {
    const val = parseFloat(e.target.value);
    sim.v_bus = val;
    dispVbus.textContent = val.toFixed(1) + ' V';
  });

  // Current Load Slider
  const sliderLoad = document.getElementById('slider-load');
  const dispLoad = document.getElementById('disp-load');
  sliderLoad.addEventListener('input', (e) => {
    const val = parseFloat(e.target.value);
    sim.load_current_override = val;
    dispLoad.textContent = val.toFixed(1) + ' A';
  });

  // Temperature Slider
  const sliderTemp = document.getElementById('slider-temp');
  const dispTemp = document.getElementById('disp-temp');
  sliderTemp.addEventListener('input', (e) => {
    const val = parseFloat(e.target.value);
    sim.temp_c = val;
    dispTemp.textContent = val.toFixed(1) + ' °C';
  });

  // Buttons
  document.getElementById('btn-arm-toggle').addEventListener('click', () => {
    if (sim.is_armed) disarmESC();
    else armESC();
  });

  document.getElementById('btn-kill-gates').addEventListener('click', () => {
    tripFault("ESTOP BUTTON PRESSED: Main Output Enable cleared!");
  });

  document.getElementById('btn-clear-faults').addEventListener('click', () => {
    clearFaults();
  });

  // Scope Tabs
  document.querySelectorAll('.tab-btn').forEach(btn => {
    btn.addEventListener('click', (e) => {
      document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
      e.target.classList.add('active');
      sim.scope_channel = e.target.dataset.channel;
    });
  });

  // Fault Injection Buttons
  const btnStall = document.getElementById('btn-inject-stall');
  btnStall.addEventListener('click', () => {
    sim.rotor_locked = !sim.rotor_locked;
    btnStall.textContent = sim.rotor_locked ? 'RELEASE ROTOR' : 'HOLD ROTOR (STALL)';
    btnStall.classList.toggle('btn-danger', sim.rotor_locked);
    if (sim.rotor_locked) logCLI("INJECT: Motor shaft physically locked at 0 RPM.");
    else logCLI("INJECT: Motor shaft released.");
  });

  const btnSigLoss = document.getElementById('btn-inject-sigloss');
  btnSigLoss.addEventListener('click', () => {
    sim.signal_cut = !sim.signal_cut;
    btnSigLoss.textContent = sim.signal_cut ? 'RESTORE SIGNAL' : 'CUT SIGNAL (WATCHDOG)';
    btnSigLoss.classList.toggle('btn-danger', sim.signal_cut);
    if (sim.signal_cut) logCLI("INJECT: Throttle line disconnected.");
    else logCLI("INJECT: Throttle line reconnected.");
  });

  document.getElementById('btn-bootloader-jump').addEventListener('click', () => {
    triggerBootloaderJump();
  });

  document.getElementById('btn-close-modal').addEventListener('click', () => {
    document.getElementById('modal-bootloader').classList.add('hidden');
    sim.state = FOC_STATE.DISARMED;
    sim.is_armed = false;
    logCLI("Rebooted to user firmware. System in standby.");
  });

  // CLI Input
  const cliInput = document.getElementById('cli-input');
  const btnCliSend = document.getElementById('btn-cli-send');

  function handleSendCLI() {
    const txt = cliInput.value;
    if (txt) {
      parseCLICommand(txt);
      cliInput.value = '';
    }
  }

  btnCliSend.addEventListener('click', handleSendCLI);
  cliInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') handleSendCLI();
  });

  document.getElementById('btn-cli-clear').addEventListener('click', () => {
    document.getElementById('cli-output').innerHTML = '';
  });

  // Flasher Modal Event Listeners
  const modalFlasher = document.getElementById('modal-flasher');
  const btnOpenFlasher = document.getElementById('btn-open-flasher');
  const btnCloseFlasher = document.getElementById('btn-close-flasher');
  const btnCloseFlasherTop = document.getElementById('btn-close-flasher-top');
  const btnStartFlash = document.getElementById('btn-start-flash');
  const btnMassErase = document.getElementById('btn-mass-erase');

  btnOpenFlasher.addEventListener('click', () => {
    modalFlasher.classList.remove('hidden');
  });

  const closeFlasher = () => {
    modalFlasher.classList.add('hidden');
  };
  btnCloseFlasher.addEventListener('click', closeFlasher);
  btnCloseFlasherTop.addEventListener('click', closeFlasher);

  // Virtual IC Flashing Animation Routine
  btnStartFlash.addEventListener('click', () => {
    btnStartFlash.disabled = true;
    btnMassErase.disabled = true;

    const term = document.getElementById('flasher-terminal');
    const pFill = document.getElementById('flash-progress-fill');
    const pLabel = document.getElementById('flash-progress-label');
    const icChip = document.getElementById('ic-chip-silicon');
    const pinSwdio = document.getElementById('pin-swdio');
    const pinSwclk = document.getElementById('pin-swclk');
    const pinNrst = document.getElementById('pin-nrst');

    icChip.classList.add('programming');
    pinSwdio.classList.add('pin-pulse');
    pinSwclk.classList.add('pin-pulse');

    function logTerm(msg, isSuccess = false) {
      const line = document.createElement('div');
      line.className = 'term-line ' + (isSuccess ? 'highlight-green' : '');
      line.textContent = msg;
      term.appendChild(line);
      term.scrollTop = term.scrollHeight;
    }

    term.innerHTML = '';
    logTerm('>>> INITIATING SWD HARDWARE PROGRAMMING SEQUENCE <<<');
    pFill.style.width = '10%';
    pLabel.textContent = 'Connecting SWD Debug Port @ 4.0 MHz...';

    setTimeout(() => {
      logTerm('[1/5] Target Detected: STM32G431CBU6 (DevID: 0x468, CoreID: 0x2BA01477)');
      logTerm('      Supply: VDD = 3.31V | Flash: 128 KB (64 pages) | SRAM: 32 KB');
      pFill.style.width = '25%';
      pLabel.textContent = 'Unlocking ST Embedded Flash Controller...';

      setTimeout(() => {
        logTerm('[2/5] Writing Flash Keys: 0x45670123, 0xCDEF89AB -> FLASH->KEYR');
        logTerm('      Flash Controller Unlocked. FLASH_SR: BSY=0, PGERR=0, WRPERR=0');
        pFill.style.width = '45%';
        pLabel.textContent = 'Mass Erasing 64 Pages (0x08000000 - 0x08020000)...';

        setTimeout(() => {
          logTerm('[3/5] Mass Erase Completed. All sectors set to 0xFF.');
          pFill.style.width = '70%';
          pLabel.textContent = 'Writing Firmware Binary (36,864 bytes @ 0x08000000)...';

          setTimeout(() => {
            logTerm('[4/5] Programming Firmware Binary complete (36 KB).');
            logTerm('      Source Checksum: CRC-32 = 0xC2D65459');
            pFill.style.width = '90%';
            pLabel.textContent = 'Executing Read-Back Byte-by-Byte Verification...';

            setTimeout(() => {
              logTerm('[5/5] Verification: Read-Back CRC-32 = 0xC2D65459 -> 100% EXACT MATCH (0 ERRORS)!', true);
              pinNrst.classList.add('pin-pulse');
              logTerm('      Asserting Hardware NRST low pulse -> Target MCU rebooting into firmware...', true);

              setTimeout(() => {
                pFill.style.width = '100%';
                pLabel.textContent = 'Flashing Complete! Virtual STM32G431 is Running in Thread Mode.';
                logTerm('>>> FIRMWARE RUNNING: FPU Active | 24kHz FOC Loop Armed | SysTick 1kHz Active <<<', true);

                icChip.classList.remove('programming');
                pinSwdio.classList.remove('pin-pulse');
                pinSwclk.classList.remove('pin-pulse');
                pinNrst.classList.remove('pin-pulse');

                btnStartFlash.disabled = false;
                btnMassErase.disabled = false;

                // Reset simulator into freshly calibrated state
                sim.state = FOC_STATE.DISARMED;
                sim.is_armed = false;
                sim.active_faults = FAULT_FLAGS.NONE;
                logCLI("VIRTUAL IC PROGRAMMED: STM32G431 flashed with v1.0.0. Inverter standby.");
              }, 400);
            }, 500);
          }, 600);
        }, 500);
      }, 400);
    }, 400);
  });

  // Mass Erase Button
  btnMassErase.addEventListener('click', () => {
    const term = document.getElementById('flasher-terminal');
    const pFill = document.getElementById('flash-progress-fill');
    const pLabel = document.getElementById('flash-progress-label');
    const hexView = document.getElementById('hex-dump-view');

    term.innerHTML = '<div class="term-line">Mass Erasing 128KB flash memory...</div>';
    pFill.style.width = '100%';
    pLabel.textContent = 'Flash Memory Erased (All bytes = 0xFF).';

    hexView.textContent = 
`0x08000000:  FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF  |................| (Erased State)
0x08000010:  FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF  |................|
0x08000020:  FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF  |................|
0x08000030:  FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF  |................|`;

    sim.state = FOC_STATE.CALIBRATION;
    sim.is_armed = false;
    logCLI("FLASH ERASED: Chip memory cleared to 0xFF. Firmware halted.");
  });

  // Start Animation
  requestAnimationFrame(simulationLoop);
});
