/*
 * vcu_gateway.c
 * RT-Thread VCU gateway module.
 *
 * Threads:
 *  - sbus_thread: SBUS decode/filter/mix -> rc_intent update
 *  - fsm_thread : source arbitration -> motor_cmd + upper_status update
 *  - can_rx_thread: CAN RX parse -> latest caches
 *  - can_tx_thread: periodic control TX + upper status/report TX
 *
 * Key CAN IDs (current):
 *  - Upper -> Gateway CMD:   0x18FF0200, 0x18FF0210, 0x18FF0220, 0x18FF0230, 0x18FF0240
 *  - Gateway -> Upper ST:    0x18FF0300, 0x18FF0310, 0x18FF0320, 0x18FF0330
 *  - Gateway -> Upper MON:   0x18FF4000, 0x18FF4010
 *  - Weed actuator RX:       0x18FF00C8
 *  - Weed actuator TX:       0x18EFC800
 *  - Motor status RX:        0x18FF0021 (left), 0x18FF0020 (right)
 *  - Motor command TX:       0x18FF2100 (left), 0x18FF2000 (right)
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <rtthread.h>

#include "CAN_AGMO.h"
#include "MPU6050.h"
#include "SBUS_AGMO.h"
#include "board.h"
#include "main.h"
#include "rc_mixer.h"
#include "vcu_gateway.h"

/* ===================== Internal Types ===================== */
/* RC intent snapshot (produced by sbus_thread, consumed by fsm_thread). */
typedef struct {
  rt_tick_t ts; /* rt_tick */
  bool valid;
  bool rc_emergency_stop;    /* RC A button: emergency stop */
  bool rc_enable;            /* RC B button: rc enable flag */
  bool rc_remote_automation; /* RC D button: remote automation flag */
  bool rc_drive_mode;        /* RC C button: drive flag */
  bool cultivator_down;
  bool cultivator_on;
  uint16_t blade_rpm_cmd;  /* RC CH6 mapped blade rpm stage: 0/250/500 */
  uint16_t weed_target_mm; /* RC left toggle mapped target: 0/90/180 mm */
  int16_t axis1;           /* rc Right stick up/down */
  int16_t axis2;           /* rc Right stick left/right */
  int16_t axis3;           /* rc Left stick up/down */
  int16_t axis4;           /* rc Left stick left/right */
  int16_t left_rpm_value;  /* differential mix result for left motor */
  int16_t right_rpm_value; /* differential mix result for right motor */
  bool failsafe;
} rc_intent_t;

typedef struct {
  rt_tick_t ts;
  bool valid;
  bool automation;               /* data[0]: upper automation gate */
  bool upper_force_stop;         /* data[1]: E-stop */
  bool upper_force_active;       /* data[2]: force upper mode */
  uint8_t relay_mask;            /* data[3] */
  uint8_t left_accel_cmd;        /* data[4]: left accel (0..255) */
  uint8_t right_accel_cmd;       /* data[5]: right accel (0..255) */
} upper_intent_t;

typedef struct {
  rt_tick_t ts;
  bool valid;
  int16_t throttle_cmd;          /* upper command */
  int16_t steering_cmd;          /* upper command */
  uint16_t max_driver_input_cmd; /* data[4:5], absolute max driver input */
  uint16_t max_speed_kmh_x100;   /* data[6:7], max speed km/h * 100 */
} upper_intent_drive_t;

typedef struct {
  rt_tick_t ts;
  bool valid;
  int16_t linear_mps_x1000;   /* data[0:1], linear speed (m/s * 1000) */
  int16_t yaw_rate_deg_s_x10; /* data[2:3], yaw rate (deg/s * 10) */
} upper_intent_auto_t;

typedef struct {
  rt_tick_t ts;
  bool valid;
  uint8_t command_type;       /* data[0]: STOP/SET_TARGET/MOVE_TO_TARGET */
  uint8_t stage;              /* data[1]: 0 up, 1 mid, 2 down */
  uint16_t target_position_mm; /* data[2:3], uint16 BE */
  uint8_t option;             /* data[4], reserved option */
} upper_weed_cmd_t;

typedef struct {
  rt_tick_t ts;
  bool valid;
  uint8_t command_type;  /* data[0]: STOP/SET_RPM/RUN */
  uint8_t mode;          /* data[1]: 0 sync */
  uint16_t left_rpm_cmd; /* data[2:3], uint16 BE */
  uint16_t right_rpm_cmd;/* data[4:5], uint16 BE */
} upper_blade_cmd_t;

typedef struct {
  rt_tick_t ts;
  bool valid;
  uint8_t fault_bits; /* 0 = ok */
  uint8_t temperature;
  int16_t rpm_axis1;
  int16_t rpm_axis2;
  int16_t supply_volt;
} motor_status_t;

typedef struct {
  rt_tick_t ts;
  bool valid;
  uint8_t fault_bits;
  uint8_t temperature;
  int16_t rpm_axis1;
  int16_t rpm_axis2;
  int16_t supply_volt;
} blade_status_t;

typedef struct {
  rt_tick_t ts;
  bool valid;
  uint16_t position_x10_mm; /* byte0-1, little-endian, 0.1mm */
  uint8_t current_raw;      /* byte2, 0.25A/bit */
  uint8_t status_flags;     /* byte3 */
  uint8_t error_code;       /* byte4 */
  uint16_t speed_x10_mm_s;  /* byte5-6, little-endian, 0.1mm/s */
  uint8_t input_state;      /* byte7 */
} weed_actuator_status_t;

typedef struct {
  /* pending=true means "prepared by FSM, not sent yet by TX thread". */
  bool pre_pending;
  bool dir_pending;
  bool pos_pending;
  uint8_t pre_frame[8];
  uint8_t dir_frame[8];
  uint8_t pos_frame[8];
  uint8_t pre_dlc;
  uint8_t dir_dlc;
  uint8_t pos_dlc;
  uint16_t target_mm;
  bool pre_sent;
  bool rx_timeout;
} weed_tx_plan_t;

typedef enum {
  WEED_SEQ_IDLE = 0,
  WEED_SEQ_WAIT_PRE1_SENT,
  WEED_SEQ_WAIT_DIR_SENT,
  WEED_SEQ_WAIT_PRE2_SENT,
  WEED_SEQ_POS_STREAM,
} weed_seq_state_t;

typedef struct {
  bool pre_sent;
  bool target_latched;
  uint16_t target_active_mm;
  bool move_is_down;
  weed_seq_state_t seq_state;
  rt_tick_t pre_guard_start_tick;
  rt_tick_t move_window_start_tick;
  rt_tick_t last_pos_tx_tick;
} weed_fsm_ctx_t;

typedef struct {
  uint16_t rpm_cmd; /* stage command from RC mapping: 0/250/500 */
} blade_cmd_t;

typedef struct {
  bool left_pending;
  bool right_pending;
  uint8_t left_frame[8];
  uint8_t right_frame[8];
  uint8_t left_dlc;
  uint8_t right_dlc;
} blade_tx_plan_t;

typedef struct {
  rt_tick_t ts;
  cmd_src_t src;
  cmd_type_t type;
  uint8_t enable_bit;
  uint8_t axis1_accel_bit;
  uint8_t axis2_accel_bit;
  int16_t rpm_axis1;
  int16_t rpm_axis2;
} motor_cmd_t;

typedef struct {
  rt_tick_t ts;
  fsm_control_src_t control_src;
  vcu_fsm_mode_t fsm_mode;
  fsm_stop_reason_t stop_reason;
  uint8_t rc_status_mask;      /* RC status bit mask */
  uint8_t fsm_status_mask;     /* FSM status bit mask (0x18FF0310 data[5]) */
  int16_t power_supply_value;  /* data[0:1] for 0x18FF0310 */
  uint8_t md_left_fault_msg;   /* motor driver left */
  uint8_t md_right_fault_msg;  /* motor driver right */
  uint8_t relay_st;            /* Bit mask */
  uint8_t timeout_detail_code; /* data[7]: timeout detail code */

} upper_status_t;

typedef struct {
  rt_tick_t ts;
  int16_t driver_left_axis1_rpm;
  int16_t driver_left_axis2_rpm;
  int16_t driver_right_axis1_rpm;
  int16_t driver_right_axis2_rpm;
} upper_status_rpm_t;

/* CAN frame for RX queue */
typedef struct {
  uint32_t ext_id;
  uint8_t dlc;
  uint8_t data[8];
} can_frame_t;

/* ===================== Shared State ===================== */
/* Latest mailbox snapshots shared across threads (protected by g_lock). */
struct {
  rc_intent_t rc;
  upper_intent_t upper_cmd_config;
  upper_intent_drive_t upper_cmd_drive;
  upper_intent_auto_t upper_cmd_auto;
  upper_weed_cmd_t upper_cmd_weed;
  upper_blade_cmd_t upper_cmd_blade;
  motor_cmd_t motor_cmd_left;
  motor_cmd_t motor_cmd_right;

  motor_status_t motor_left;
  motor_status_t motor_right;
  blade_status_t blade_left;
  blade_status_t blade_right;
  blade_cmd_t blade_cmd;
  blade_tx_plan_t blade_tx_plan;
  weed_actuator_status_t weed_actuator;
  weed_tx_plan_t weed_tx_plan;
  upper_status_t upper_vcu_st;
  upper_status_rpm_t upper_rpm_st;
  vcu_motion_monitor_t motion_monitor;
} g_latest;

static rt_mutex_t g_lock = RT_NULL;

/* ===================== CAN RX message queue ===================== */
static rt_mq_t g_can_rx_mq = RT_NULL;
#define CAN_RX_MQ_DEPTH 32

/* ===================== Module Scope State ===================== */
/* Mixer debug state snapshot from SBUS path (for diagnostics/monitoring). */
calc_state_t rc_mix_state;
static weed_fsm_ctx_t g_weed_fsm_ctx;

/* Legacy debug symbols kept for compatibility with existing debug paths. */
int16_t rpm_a;
SBUS_CH_DATA sbus_data_raw_a;
static float AccGyroValue[6]; /* MPU6050 shared buffer: [3..5] gyro raw */

/* ===================== Helpers ===================== */
static inline rt_tick_t now_tick(void) {
  return rt_tick_get();
}

static inline bool is_fresh_tick(rt_tick_t now, rt_tick_t ts, uint32_t timeout_ms) {
  if (ts == 0)
    return false;
  rt_tick_t dt = now - ts;
  uint32_t dt_ms = (uint32_t)(dt * 1000 / RT_TICK_PER_SECOND);
  return dt_ms < timeout_ms;
}

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

static inline int32_t rcm_max_rc_i32(void) {
  return (int32_t)g_rcm_vehicle.max_rc_input; /* expected 500 */
}

static inline int32_t rcm_max_driver_i32(void) {
  return (int32_t)g_rcm_vehicle.max_driver_input; /* expected 664 */
}

static int16_t sbus_to_cmd(int16_t sbus_data) {
  int32_t max_rc = rcm_max_rc_i32();
  sbus_data = (int16_t)clamp_i32(sbus_data, SBUS_MIN, SBUS_MAX);

  if (sbus_data > (SBUS_CENTER - DEADBAND) && sbus_data < (SBUS_CENTER + DEADBAND)) {
    return 0;
  }
  if (sbus_data >= SBUS_CENTER) {
    int32_t num = (int32_t)(sbus_data - SBUS_CENTER) * max_rc;
    int32_t den = (SBUS_MAX - SBUS_CENTER);
    return (int16_t)((num / den));
  } else {
    int32_t num = (int32_t)(SBUS_CENTER - sbus_data) * max_rc;
    int32_t den = (SBUS_CENTER - SBUS_MIN);
    return (int16_t)(-(num / den));
  }
}

// int16 -> Data1(high), Data2(low) (bin-endian packing)

static inline void pack_int16_hi_lo(int16_t v, uint8_t* hi, uint8_t* lo) {
  uint16_t u = (uint16_t)v;
  *hi = (uint8_t)(u >> 8);
  *lo = (uint8_t)(u & 0xFF);
}

static inline int16_t clamp_to_i16(int32_t v) {
  if (v > 32767)
    return 32767;
  if (v < -32768)
    return -32768;
  return (int16_t)v;
}

static inline uint8_t clamp_u16_to_u8(uint16_t v) {
  return (uint8_t)((v > 255u) ? 255u : v);
}

static uint16_t map_ch5_to_weed_target_mm(uint16_t ch5_raw) {
  uint32_t d_down;
  uint32_t d_mid;
  uint32_t d_up;

  d_down = (ch5_raw >= WEED_CH5_RAW_DOWN) ? (uint32_t)(ch5_raw - WEED_CH5_RAW_DOWN)
                                          : (uint32_t)(WEED_CH5_RAW_DOWN - ch5_raw);
  d_mid =
      (ch5_raw >= WEED_CH5_RAW_MID) ? (uint32_t)(ch5_raw - WEED_CH5_RAW_MID) : (uint32_t)(WEED_CH5_RAW_MID - ch5_raw);
  d_up = (ch5_raw >= WEED_CH5_RAW_UP) ? (uint32_t)(ch5_raw - WEED_CH5_RAW_UP) : (uint32_t)(WEED_CH5_RAW_UP - ch5_raw);

  if (d_down <= d_mid && d_down <= d_up)
    return WEED_POS_DOWN_MM;
  if (d_mid <= d_down && d_mid <= d_up)
    return WEED_POS_MID_MM;
  return WEED_POS_UP_MM;
}

static uint16_t map_upper_weed_stage_to_mm(uint8_t stage_or_mm) {
  if (stage_or_mm == UPPER_WEED_STAGE_UP)
    return WEED_POS_UP_MM;
  if (stage_or_mm == UPPER_WEED_STAGE_MID)
    return WEED_POS_MID_MM;
  if (stage_or_mm == UPPER_WEED_STAGE_DOWN)
    return WEED_POS_DOWN_MM;
  return (uint16_t)clamp_i32((int32_t)stage_or_mm, 0, WEED_ACT_POS_MAX_MM);
}

static uint16_t map_ch6_to_blade_rpm(uint16_t ch6_raw) {
  uint32_t d_low;
  uint32_t d_mid;
  uint32_t d_high;

  d_low = (ch6_raw >= BLADE_CH6_RAW_LOW) ? (uint32_t)(ch6_raw - BLADE_CH6_RAW_LOW)
                                         : (uint32_t)(BLADE_CH6_RAW_LOW - ch6_raw);
  d_mid = (ch6_raw >= BLADE_CH6_RAW_MID) ? (uint32_t)(ch6_raw - BLADE_CH6_RAW_MID)
                                         : (uint32_t)(BLADE_CH6_RAW_MID - ch6_raw);
  d_high = (ch6_raw >= BLADE_CH6_RAW_HIGH) ? (uint32_t)(ch6_raw - BLADE_CH6_RAW_HIGH)
                                           : (uint32_t)(BLADE_CH6_RAW_HIGH - ch6_raw);

  if (d_low <= d_mid && d_low <= d_high)
    return BLADE_RPM_STAGE_LOW;
  if (d_mid <= d_low && d_mid <= d_high)
    return BLADE_RPM_STAGE_MID;
  return BLADE_RPM_STAGE_HIGH;
}

/* MPU6050 gyro config is set to +-2000dps (GYRO_CONFIG=0x18): 16.4 LSB/(deg/s). */
static bool read_mpu_gyro_z_deg_s(float* out_deg_s) {
  float raw;
  if (!out_deg_s)
    return false;
  MPU6050_ReadData(AccGyroValue);
  raw = AccGyroValue[1];
  if (raw > 32767.0f || raw < -32768.0f)
    return false;
  *out_deg_s = raw / 16.4f;
  return true;
}

static uint8_t make_timeout_detail_code(bool rc_timeout, bool upper_cfg_timeout, bool upper_drive_timeout,
                                        bool motor_left_timeout, bool motor_right_timeout) {
  uint8_t code = TO_NONE;
  uint8_t cnt = 0;

  if (rc_timeout) {
    code = TO_RC;
    cnt++;
  }
  if (upper_cfg_timeout) {
    code = TO_UPPER_CFG;
    cnt++;
  }
  if (upper_drive_timeout) {
    code = TO_UPPER_DRIVE;
    cnt++;
  }
  if (motor_left_timeout) {
    code = TO_MOTOR_LEFT;
    cnt++;
  }
  if (motor_right_timeout) {
    code = TO_MOTOR_RIGHT;
    cnt++;
  }

  if (cnt > 1u)
    return TO_MULTIPLE;
  return code;
}

/* Auto handover gate:
 * Upper auto control is allowed only when both sides agree.
 * - RC side: operator enables remote automation while RC link is active.
 * - Upper side: config automation flag is enabled (valid frame received).
 */
static bool is_upper_auto_handover_ready(bool rc_active, bool rc_remote_automation, bool upper_cfg_valid,
                                         bool upper_automation) {
  return (rc_active && rc_remote_automation && upper_cfg_valid && upper_automation);
}

static vcu_fsm_mode_t decide_fsm_mode(bool rc_active, bool rc_remote_automation, bool upper_auto_ready,
                                      fsm_control_src_t control_src, fsm_stop_reason_t stop_reason) {
  if (control_src == FSM_CTRL_SRC_STOP || stop_reason != FSM_STOP_REASON_NONE)
    return FSM_MODE_SAFE_STOP;
  if (control_src == FSM_CTRL_SRC_UPPER_AUTO)
    return FSM_MODE_AUTO_ACTIVE;
  if (control_src == FSM_CTRL_SRC_UPPER)
    return FSM_MODE_AUTO_ACTIVE;
  if (rc_active && rc_remote_automation && !upper_auto_ready)
    return FSM_MODE_AUTO_ARMED;
  if (control_src == FSM_CTRL_SRC_RC)
    return FSM_MODE_MANUAL_RC;
  return FSM_MODE_SAFE_STOP;
}

static uint8_t pack_fsm_status_mask(vcu_fsm_mode_t mode, fsm_stop_reason_t reason) {
  uint8_t mask = 0;
  switch (mode) {
  case FSM_MODE_SAFE_STOP: mask |= FSM_ST_MODE_SAFE_STOP; break;
  case FSM_MODE_MANUAL_RC: mask |= FSM_ST_MODE_MANUAL_RC; break;
  case FSM_MODE_AUTO_ARMED: mask |= FSM_ST_MODE_AUTO_ARMED; break;
  case FSM_MODE_AUTO_ACTIVE: mask |= FSM_ST_MODE_AUTO_ACTIVE; break;
  default: mask |= FSM_ST_MODE_SAFE_STOP; break;
  }

  if (reason == FSM_STOP_UPPER_FORCE)
    mask |= FSM_ST_STOP_UPPER_FORCE;
  else if (reason == FSM_STOP_RC_EMG)
    mask |= FSM_ST_STOP_RC_EMG;
  else if (reason == FSM_STOP_MOTOR_FAULT)
    mask |= FSM_ST_STOP_MOTOR_FAULT;
  else if (reason == FSM_STOP_TIMEOUT)
    mask |= FSM_ST_STOP_TIMEOUT;

  return mask;
}

/* Upper auto command mixer:
 * input: linear speed (m/s), yaw-rate (deg/s)
 * output: left/right driver command in command scale.
 */
static void mix_upper_auto_cmd_to_tracks(int16_t linear_mps_x1000, int16_t yaw_rate_deg_s_x10, int16_t* left_out,
                                         int16_t* right_out) {
  float v_m_s, yaw_rate_deg_s, omega_rad_s;
  float half_track_m, wheel_circ_m;
  float v_left_m_s, v_right_m_s;
  float rpm_left, rpm_right;
  float cmd_left, cmd_right;
  float cmd_left_mapped, cmd_right_mapped;
  float max_abs, scale;
  const float driver_input_to_rpm_scale = 0.1f;
  int32_t cmd_max;

  if (!left_out || !right_out)
    return;

  v_m_s = (float)linear_mps_x1000 / 1000.0f;
  yaw_rate_deg_s = (float)yaw_rate_deg_s_x10 / 10.0f;
  omega_rad_s = yaw_rate_deg_s * (3.1415926f / 180.0f);
  half_track_m = g_rcm_vehicle.track_width_m * 0.5f;
  wheel_circ_m = 3.1415926f * g_rcm_vehicle.wheel_diameter_m;

  v_left_m_s = v_m_s - (omega_rad_s * half_track_m);
  v_right_m_s = v_m_s + (omega_rad_s * half_track_m);

  rpm_left = (v_left_m_s * 60.0f) / wheel_circ_m;
  rpm_right = (v_right_m_s * 60.0f) / wheel_circ_m;

  /* driver_input ~= rpm / 0.1 */
  cmd_left = rpm_left / driver_input_to_rpm_scale;
  cmd_right = rpm_right / driver_input_to_rpm_scale;

  /* Apply geometry sign mapping (same policy as RC mixer output). */
  cmd_left_mapped = cmd_left * (float)((g_rcm_vehicle.left_dir_sign >= 0) ? 1 : -1);
  cmd_right_mapped = cmd_right * (float)((g_rcm_vehicle.right_dir_sign >= 0) ? 1 : -1);

  /* Common saturation with ratio preservation. */
  cmd_max = rcm_max_driver_i32();
  max_abs = (cmd_left_mapped >= 0.0f) ? cmd_left_mapped : -cmd_left_mapped;
  {
    float right_abs = (cmd_right_mapped >= 0.0f) ? cmd_right_mapped : -cmd_right_mapped;
    if (right_abs > max_abs)
      max_abs = right_abs;
  }
  if (max_abs > (float)cmd_max && max_abs > 0.0f) {
    scale = (float)cmd_max / max_abs;
    cmd_left_mapped *= scale;
    cmd_right_mapped *= scale;
  }

  *left_out = (int16_t)clamp_i32((int32_t)cmd_left_mapped, -cmd_max, cmd_max);
  *right_out = (int16_t)clamp_i32((int32_t)cmd_right_mapped, -cmd_max, cmd_max);
}

/* Optional AUTO mode path:
 * Convert (linear speed, yaw-rate) to pseudo RC throttle/steering, then reuse RC mixer.
 */
static void mix_upper_auto_cmd_via_rc_mixer(int16_t linear_mps_x1000, int16_t yaw_rate_deg_s_x10, int16_t* left_out,
                                            int16_t* right_out) {
  rc_input_t auto_as_rc;
  vehicle_config_t cfg;
  tune_config_t tune;
  calc_state_t st;
  motor_output_t out;
  float v_m_s, yaw_rate_deg_s, omega_rad_s;
  float max_speed_m_s, omega_max_rad_s;
  float throttle_norm, steering_norm;

  if (!left_out || !right_out)
    return;

  cfg = g_rcm_vehicle;
  tune = g_rcm_tune;
  memset(&st, 0, sizeof(st));

  v_m_s = (float)linear_mps_x1000 / 1000.0f;
  yaw_rate_deg_s = (float)yaw_rate_deg_s_x10 / 10.0f;
  omega_rad_s = yaw_rate_deg_s * (3.1415926f / 180.0f);

  max_speed_m_s = cfg.max_speed_kmh / 3.6f;
  if (max_speed_m_s <= 0.001f || cfg.track_width_m <= 0.001f) {
    *left_out = 0;
    *right_out = 0;
    return;
  }

  /* omega = 2 * vc * beta / track_width => omega_max at vc=max_speed, beta=1 */
  omega_max_rad_s = (2.0f * max_speed_m_s) / cfg.track_width_m;
  if (omega_max_rad_s <= 0.001f)
    omega_max_rad_s = 0.001f;

  throttle_norm = v_m_s / max_speed_m_s;
  steering_norm = omega_rad_s / omega_max_rad_s;
  if (throttle_norm > 1.0f)
    throttle_norm = 1.0f;
  if (throttle_norm < -1.0f)
    throttle_norm = -1.0f;
  if (steering_norm > 1.0f)
    steering_norm = 1.0f;
  if (steering_norm < -1.0f)
    steering_norm = -1.0f;

  auto_as_rc.throttle = throttle_norm * cfg.max_rc_input;
  auto_as_rc.steering = steering_norm * cfg.max_rc_input;
  out = mix_rc_to_tracks(&auto_as_rc, false, &cfg, &tune, &st);

  *left_out = out.left_input;
  *right_out = out.right_input;
}

static int16_t sbus_convert_to_control(int16_t sbus_data, uint8_t data[2]) {
  int16_t value = sbus_to_cmd(sbus_data);
  pack_int16_hi_lo(value, &data[1], &data[0]);
  return value;
}

/* SBUS moving-average filter for axis smoothing */
#define SBUS_FILTER_WINDOW 10u
typedef struct {
  int16_t buf[SBUS_FILTER_WINDOW];
  int32_t sum;
  uint8_t idx;
  uint8_t count;
} sbus_ma_filter_t;

static void sbus_ma_reset(sbus_ma_filter_t* f) {
  if (!f)
    return;
  memset(f, 0, sizeof(*f));
}

static int16_t sbus_ma_update(sbus_ma_filter_t* f, int16_t sbus_data) {
  if (!f)
    return sbus_data;

  if (f->count < SBUS_FILTER_WINDOW) {
    f->buf[f->idx] = sbus_data;
    f->sum += sbus_data;
    f->idx = (uint8_t)((f->idx + 1u) % SBUS_FILTER_WINDOW);
    f->count++;
    return (int16_t)(f->sum / (int32_t)f->count);
  }

  f->sum -= f->buf[f->idx];
  f->buf[f->idx] = sbus_data;
  f->sum += sbus_data;
  f->idx = (uint8_t)((f->idx + 1u) % SBUS_FILTER_WINDOW);
  return (int16_t)(f->sum / (int32_t)SBUS_FILTER_WINDOW);
}

/* Differential-drive mixer (current behavior)
 * 1) Base mix:
 *    left  = throttle + steering
 *    right = throttle - steering
 *
 * 2) Direction-reversal guard while driving:
 *    If throttle != 0, steering magnitude is capped to |throttle|.
 *    => During forward/backward driving, one side will not flip to reverse.
 *
 * 3) In-place turn is allowed:
 *    If throttle == 0, steering cap is not applied.
 *
 * 4) Final output clamp:
 *    left/right are clamped to +/- RCM_MAX_DRIVER_INPUT.
 */

void vcu_diff_drive_mix(int16_t throttle, int16_t steering, int16_t* left, int16_t* right) {
  int32_t cmd_min = -rcm_max_driver_i32();
  int32_t cmd_max = rcm_max_driver_i32();
  int32_t t = (int32_t)throttle;
  int32_t s = (int32_t)steering;
  // int32_t t = (int32_t)steering;
  // int32_t s = (int32_t)throttle;
  int32_t t_abs, s_abs;
  int32_t outer, inner;

  // int32_t t_abs = (t >= 0) ? t : -t;
  // int32_t s_abs = (s >= 0) ? s : -s;

  t_abs = (t >= 0) ? t : -t;
  s_abs = (s <= 0) ? -s : s;

  // rt_kprintf("streeing data : %d! \n", s);
  // rt_kprintf("throttle data : %d! \n", t);
  // rt_kprintf("t_abs data : %d! \n", t_abs);
  // rt_kprintf("s_abs data : %d! \n", s_abs);

  if (!left || !right)
    return;

  if (t_abs > 0 && s_abs > t_abs)
    s_abs = t_abs;

  if (t == 0) {
    // rt_kprintf("streeing data : %d! \n", s);
    *left = (int16_t)clamp_i32(s, cmd_min, cmd_max);
    *right = (int16_t)clamp_i32(s, cmd_min, cmd_max);
    return;
  }

  outer = t;
  // t=400
  // s_abs = 100
  //  (400 * (400 - 100)) / 400

  inner = (t * (t_abs - s_abs)) / t_abs;
  // inner = (t * (t_abs + s_abs)) / t_abs;

  // rt_kprintf("outer data : %d! \n", outer);
  // rt_kprintf("inner data : %d! \n", inner);

  if (s < 0) {
    *right = (int16_t)clamp_i32(-outer, cmd_min, cmd_max);
    *left = (int16_t)clamp_i32(inner, cmd_min, cmd_max);
  } else if (s > 0) {
    *left = (int16_t)clamp_i32(outer, cmd_min, cmd_max);
    *right = (int16_t)clamp_i32(-inner, cmd_min, cmd_max);
  } else {
    *left = (int16_t)clamp_i32(t, cmd_min, cmd_max);
    *right = (int16_t)clamp_i32(-t, cmd_min, cmd_max);
  }
}

/* ===================== RC Mixer / Monitor ===================== */
#define PI_F                      (3.1415926f)
#define DRIVER_INPUT_TO_RPM_SCALE (0.1f) /* driver input ~= rpm * 10 */

static void update_motion_monitor(vcu_motion_monitor_t* mon, rt_tick_t now_tick_value, int16_t left_input,
                                  int16_t right_input) {
  /* NOTE:
   * This monitor is currently command-based (out_cmd), not measured-RPM based.
   * left_input/right_input should be commanded driver inputs.
   */
  float left_rpm, right_rpm;
  float wheel_circumference_m;
  float left_speed, right_speed, center_speed;
  float yaw_rate_rad_s, yaw_rate_deg_s;
  float dt_s = 0.0f;

  if (!mon)
    return;

  if (mon->valid && mon->ts_tick != 0 && now_tick_value > (rt_tick_t)mon->ts_tick)
    dt_s = (float)(now_tick_value - (rt_tick_t)mon->ts_tick) / (float)RT_TICK_PER_SECOND;
  else
    dt_s = (float)FSM_PERIOD_MS / 1000.0f;

  left_rpm = (float)left_input * DRIVER_INPUT_TO_RPM_SCALE;
  right_rpm = (float)right_input * DRIVER_INPUT_TO_RPM_SCALE;
  wheel_circumference_m = PI_F * g_rcm_vehicle.wheel_diameter_m;

  left_speed = (left_rpm * wheel_circumference_m) / 60.0f;
  right_speed = (right_rpm * wheel_circumference_m) / 60.0f;
  center_speed = 0.5f * (left_speed + right_speed);

  yaw_rate_rad_s = (right_speed - left_speed) / g_rcm_vehicle.track_width_m;
  yaw_rate_deg_s = yaw_rate_rad_s * 57.2957795f;

  mon->left_distance_m += left_speed * dt_s;
  mon->right_distance_m += right_speed * dt_s;
  mon->center_distance_m += center_speed * dt_s;

  mon->yaw_deg_0_360 += yaw_rate_deg_s * dt_s;
  while (mon->yaw_deg_0_360 >= 360.0f)
    mon->yaw_deg_0_360 -= 360.0f;
  while (mon->yaw_deg_0_360 < 0.0f)
    mon->yaw_deg_0_360 += 360.0f;

  mon->left_driver_input = left_input;
  mon->right_driver_input = right_input;
  mon->left_speed_m_s = left_speed;
  mon->right_speed_m_s = right_speed;
  mon->center_speed_m_s = center_speed;
  mon->yaw_rate_deg_s = yaw_rate_deg_s;
  mon->imu_yaw_rate_valid = read_mpu_gyro_z_deg_s(&mon->imu_yaw_rate_deg_s);
  mon->ts_tick = (uint32_t)now_tick_value;
  mon->valid = true;
}

static void make_can_payload_from_sbus(int16_t sbus_data, uint8_t data[8]) {
  int16_t cmd = sbus_to_cmd(sbus_data);
  data[0] = 0xE3;
  pack_int16_hi_lo(cmd, &data[1], &data[2]);
  data[3] = 0x32;
  pack_int16_hi_lo(cmd, &data[4], &data[5]);
  data[6] = 0x32;
  data[7] = 0x0;
}

/* ===================== Hardware Abstraction Stubs ===================== */
/* TODO: Replace with your actual CAN send function (HAL_CAN_AddTxMessage or rt_device_can write) */
static bool can_hw_send_ext(uint32_t ext_id, const uint8_t data[8], uint8_t dlc) {
  //(void)ext_id; (void)data; (void)dlc;

  /* TODO implement */
  return can_send_ext(ext_id, data, dlc);
}

/* TODO: Replace with SBUS decoder */
static bool sbus_get_frame_25b(uint8_t out_frame[25]) {
  (void)out_frame;
  /* TODO implement: block/wait or poll from UART DMA ring buffer */
  return get_decode_25b_data(out_frame);
}
static bool sbus_decode_25b_to_channels(uint8_t frame[25], SBUS_CH_DATA* ch_out, bool* failsafe, bool* lost) {
  bool ret = false;
  if (!frame || !ch_out || !failsafe || !lost)
    return false;

  ret = get_decode_ch_data(frame, ch_out);

  /* rc disable(not receive) */
  if (ch_out->ConnectState == 0)
    *failsafe = true;
  //*failsafe = true;
  //*lost = true;
  /* TODO implement */
  return ret;
}

/* ===================== Packing/Decoding ===================== */
/* Upper config CMD: config-only, no actuator/blade target payload here. */
static bool decode_upper_cmd(const can_frame_t* rx, upper_intent_t* out) {
  if (rx->ext_id != CANID_UPPER_CMD_CONFIG_RX)
    return false;
  if (rx->dlc < 8u)
    return false;

  memset(out, 0, sizeof(*out));
  out->ts = now_tick();
  out->valid = true;

  /* Upper -> gateway config payload (0x18FF0210):
   * data[0]: automation
   * data[1]: E-stop
   * data[2]: force upper mode
   * data[3]: relay mask
   * data[4]: left accel command (0..255)
   * data[5]: right accel command (0..255)
   * data[6:7]: reserved
   */
  out->automation = ((rx->data[0] & 0x01u) != 0u);
  out->upper_force_stop = ((rx->data[1] & 0x01u) != 0u);
  out->upper_force_active = ((rx->data[2] & 0x01u) != 0u);
  out->relay_mask = (uint8_t)rx->data[3];
  out->left_accel_cmd = (uint8_t)rx->data[4];
  out->right_accel_cmd = (uint8_t)rx->data[5];

  return true;
}

static bool decode_upper_weed_actuator_cmd(const can_frame_t* rx, upper_weed_cmd_t* out) {
  if (rx->ext_id != CANID_UPPER_CMD_WEED_RX)
    return false;
  if (rx->dlc < 8u)
    return false;

  memset(out, 0, sizeof(*out));
  out->ts = now_tick();
  out->valid = true;

  /* Upper -> gateway weed actuator payload (0x18FF0230):
   * data[0]   : command_type (0 stop, 1 set target, 2 move to target)
   * data[1]   : stage (0 up, 1 mid, 2 down)
   * data[2:3] : target_position_mm (uint16 BE)
   * data[4]   : option
   * data[5:7] : reserved
   */
  out->command_type = rx->data[0];
  out->stage = rx->data[1];
  out->target_position_mm = (uint16_t)(((uint16_t)rx->data[2] << 8) | (uint16_t)rx->data[3]);
  out->target_position_mm = (uint16_t)clamp_i32((int32_t)out->target_position_mm, 0, WEED_ACT_POS_MAX_MM);
  out->option = rx->data[4];

  return true;
}

static bool decode_upper_blade_cmd(const can_frame_t* rx, upper_blade_cmd_t* out) {
  if (rx->ext_id != CANID_UPPER_CMD_BLADE_RX)
    return false;
  if (rx->dlc < 8u)
    return false;

  memset(out, 0, sizeof(*out));
  out->ts = now_tick();
  out->valid = true;

  /* Upper -> gateway blade payload (0x18FF0240):
   * data[0]   : command_type (0 stop, 1 set rpm, 2 run)
   * data[1]   : mode (0 sync)
   * data[2:3] : left_blade_rpm (uint16 BE, 0..2000)
   * data[4:5] : right_blade_rpm (uint16 BE, 0..2000)
   * data[6:7] : reserved
   */
  out->command_type = rx->data[0];
  out->mode = rx->data[1];
  out->left_rpm_cmd = (uint16_t)(((uint16_t)rx->data[2] << 8) | (uint16_t)rx->data[3]);
  out->right_rpm_cmd = (uint16_t)(((uint16_t)rx->data[4] << 8) | (uint16_t)rx->data[5]);
  out->left_rpm_cmd = (uint16_t)clamp_i32((int32_t)out->left_rpm_cmd, 0, BLADE_RPM_CMD_MAX);
  out->right_rpm_cmd = (uint16_t)clamp_i32((int32_t)out->right_rpm_cmd, 0, BLADE_RPM_CMD_MAX);

  return true;
}

/* Example: decode upper rpm cmd payload (adjust to your protocol) */
static bool decode_upper_drive_cmd(const can_frame_t* rx, upper_intent_drive_t* out) {
  if (rx->ext_id != CANID_UPPER_CMD_DRIVE_RX)
    return false;
  if (rx->dlc < 8u)
    return false;

  memset(out, 0, sizeof(*out));
  out->ts = now_tick();
  out->valid = false;

  /* Upper -> gateway drive payload (0x18FF0200):
   * data[0:1] : throttle_cmd (int16, signed)
   * data[2:3] : steering_cmd (int16, signed)
   * data[4:5] : max_driver_input_cmd (uint16)
   * data[6:7] : max_speed_kmh_x100 (uint16)
   */
  out->throttle_cmd = (int16_t)((uint16_t)rx->data[0] << 8 | ((uint16_t)rx->data[1]));
  out->steering_cmd = (int16_t)((uint16_t)rx->data[2] << 8 | ((uint16_t)rx->data[3]));
  out->max_driver_input_cmd = (uint16_t)(((uint16_t)rx->data[4] << 8) | ((uint16_t)rx->data[5]));
  out->max_speed_kmh_x100 = (uint16_t)(((uint16_t)rx->data[6] << 8) | ((uint16_t)rx->data[7]));
  out->throttle_cmd = (int16_t)clamp_i32((int32_t)out->throttle_cmd, -rcm_max_rc_i32(), rcm_max_rc_i32());
  out->steering_cmd = (int16_t)clamp_i32((int32_t)out->steering_cmd, -rcm_max_rc_i32(), rcm_max_rc_i32());
  out->valid = true;

  return true;
}

static bool decode_upper_auto_cmd(const can_frame_t* rx, upper_intent_auto_t* out) {
  if (rx->ext_id != CANID_UPPER_CMD_AUTO_RX)
    return false;
  if (rx->dlc < 4u)
    return false;

  memset(out, 0, sizeof(*out));
  out->ts = now_tick();
  out->valid = false;

  /* Upper -> gateway auto payload (0x18FF0220):
   * data[0:1] : linear_mps_x1000 (int16)
   * data[2:3] : yaw_rate_deg_s_x10 (int16)
   */
  out->linear_mps_x1000 = (int16_t)((uint16_t)rx->data[0] << 8 | ((uint16_t)rx->data[1]));
  out->yaw_rate_deg_s_x10 = (int16_t)((uint16_t)rx->data[2] << 8 | ((uint16_t)rx->data[3]));
  out->valid = true;

  return true;
}

/* Weed actuator status RX: 0x18FF00C8 (CYL -> VCU)
 * byte0-1: position (0.1mm), little-endian
 * byte2  : current raw (0.25A/bit)
 * byte3  : status flags
 * byte4  : error code
 * byte5-6: speed (0.1mm/s), little-endian
 * byte7  : input state
 */
static bool decode_weed_actuator_status(const can_frame_t* rx, weed_actuator_status_t* out) {
  if (rx->ext_id != CANID_WEED_ACTUATOR_STATUS_RX)
    return false;
  if (rx->dlc < 8u)
    return false;

  memset(out, 0, sizeof(*out));
  out->ts = now_tick();
  out->valid = true;
  out->position_x10_mm = (uint16_t)(((uint16_t)rx->data[1] << 8) | ((uint16_t)rx->data[0]));
  out->current_raw = rx->data[2];
  out->status_flags = rx->data[3];
  out->error_code = rx->data[4];
  out->speed_x10_mm_s = (uint16_t)(((uint16_t)rx->data[6] << 8) | ((uint16_t)rx->data[5]));
  out->input_state = rx->data[7];
  return true;
}

/* Motor status RX: 0x18FF0021 (adjust to your motor status layout) */
static bool decode_motor_status(const can_frame_t* rx, uint32_t expect_id, motor_status_t* out) {
  if (rx->ext_id != expect_id)
    return false;

  memset(out, 0, sizeof(*out));
  out->ts = now_tick();
  out->valid = true;

  /* Example:
     data[0..1]=rpm LE, data[2..3]=current LE, data[4..7]=fault_bits LE */

  out->fault_bits = (uint8_t)rx->data[0];
  out->temperature = (uint8_t)rx->data[1];
  out->rpm_axis2 = (int16_t)((uint16_t)rx->data[2] << 8 | ((uint16_t)rx->data[3]));
  out->rpm_axis1 = (int16_t)((uint16_t)rx->data[4] << 8 | ((uint16_t)rx->data[5]));
  out->supply_volt = (int16_t)((uint16_t)rx->data[6] << 8 | ((uint16_t)rx->data[7]));

  return true;
}

static bool decode_blade_status(const can_frame_t* rx, uint32_t expect_id, blade_status_t* out) {
  motor_status_t tmp;
  if (!out)
    return false;
  if (!decode_motor_status(rx, expect_id, &tmp))
    return false;

  memset(out, 0, sizeof(*out));
  out->ts = tmp.ts;
  out->valid = tmp.valid;
  out->fault_bits = tmp.fault_bits;
  out->temperature = tmp.temperature;
  out->rpm_axis1 = tmp.rpm_axis1;
  out->rpm_axis2 = tmp.rpm_axis2;
  out->supply_volt = tmp.supply_volt;
  return true;
}

/* Motor cmd TX payload (adjust to motor driver spec) */
static void pack_motor_cmd(const motor_cmd_t* cmd, uint8_t out[8]) {
  memset(out, 0, 8);
  out[0] = (uint8_t)(cmd->enable_bit & 0xFF);
  pack_int16_hi_lo(cmd->rpm_axis2, &out[1], &out[2]);
  // out[1] = (uint8_t)((cmd->rpm_axis2 >> 8) & 0xFF);
  // out[2] = (uint8_t)(cmd->rpm_axis2 & 0xFF);
  out[3] = (uint8_t)(cmd->axis2_accel_bit & 0xFF);

  pack_int16_hi_lo(cmd->rpm_axis1, &out[4], &out[5]);
  // out[4] = (uint8_t)((cmd->rpm_axis1 >> 8) & 0xFF);
  // out[5] = (uint8_t)(cmd->rpm_axis1 & 0xFF);
  out[6] = (uint8_t)(cmd->axis1_accel_bit & 0xFF);
  out[7] = (uint8_t)(0x00);
}

/* Blade command payload:
 * - left blade : single-axis valid (axis1=rpm, axis2=0)
 * - right blade: dual-axis same rpm (axis1=axis2=rpm)
 */
static void pack_blade_cmd_frame(bool left_side, uint16_t rpm_cmd, uint8_t out[8]) {
  motor_cmd_t blade_cmd;
  int16_t rpm = (int16_t)clamp_i32((int32_t)rpm_cmd, 0, BLADE_RPM_CMD_MAX);

  memset(&blade_cmd, 0, sizeof(blade_cmd));
  blade_cmd.enable_bit = BLADE_CMD_ENABLE_BITS;
  blade_cmd.axis1_accel_bit = BLADE_CMD_ACCEL;
  blade_cmd.axis2_accel_bit = BLADE_CMD_ACCEL;

  if (left_side) {
    blade_cmd.rpm_axis1 = rpm;
    blade_cmd.rpm_axis2 = rpm;
  } else {
    blade_cmd.rpm_axis1 = rpm;
    blade_cmd.rpm_axis2 = 0;
  }

  pack_motor_cmd(&blade_cmd, out);
}

/* Upper status TX 0x18FF0310 (gateway status) */
static void pack_upper_status(const upper_status_t* st, uint8_t out[8]) {
  memset(out, 0, 8);
  pack_int16_hi_lo(st->power_supply_value, &out[0], &out[1]);
  out[2] = st->md_left_fault_msg;
  out[3] = st->md_right_fault_msg;
  out[4] = st->rc_status_mask;
  out[5] = st->fsm_status_mask;
  out[6] = st->relay_st;
  out[7] = st->timeout_detail_code;
}

/* Upper status rpm TX 0x18FF0300 (motor driver feedback) */
static void pack_upper_status_rpm(const upper_status_rpm_t* rpm_fb, uint8_t out[8]) {
  memset(out, 0, 8);

  out[0] = (uint8_t)((rpm_fb->driver_left_axis1_rpm >> 8) & 0xFF);
  out[1] = (uint8_t)(rpm_fb->driver_left_axis1_rpm & 0xFF);
  out[2] = (uint8_t)((rpm_fb->driver_left_axis2_rpm >> 8) & 0xFF);
  out[3] = (uint8_t)(rpm_fb->driver_left_axis2_rpm & 0xFF);
  out[4] = (uint8_t)((rpm_fb->driver_right_axis1_rpm >> 8) & 0xFF);
  out[5] = (uint8_t)(rpm_fb->driver_right_axis1_rpm & 0xFF);
  out[6] = (uint8_t)((rpm_fb->driver_right_axis2_rpm >> 8) & 0xFF);
  out[7] = (uint8_t)(rpm_fb->driver_right_axis2_rpm & 0xFF);
  /*
    out[3] = (uint8_t)(st->axis1_cmd & 0xFF);
    out[4] = (uint8_t)((st->axis1_cmd >> 8) & 0xFF);
    out[5] = (uint8_t)(st->axis2_cmd & 0xFF);
    out[6] = (uint8_t)((st->axis2_cmd >> 8) & 0xFF);
  */
}

/* Upper vehicle status TX 0x18FF0320 (motion monitor snapshot)
 * data[0:1] : yaw_deg_0_360 * 10
 * data[2:3] : yaw_rate_deg_s * 10
 * data[4:5] : left_speed_m_s * 100
 * data[6:7] : right_speed_m_s * 100
 */
static void pack_upper_vehicle_status(const vcu_motion_monitor_t* mon, uint8_t out[8]) {
  int16_t yaw_x10, yaw_rate_x10, left_spd_x100, right_spd_x100;
  float yaw = 0.0f;
  if (!mon) {
    memset(out, 0, 8);
    return;
  }

  memset(out, 0, 8);
  yaw = mon->yaw_deg_0_360;
  while (yaw >= 360.0f)
    yaw -= 360.0f;
  while (yaw < 0.0f)
    yaw += 360.0f;

  yaw_x10 = clamp_to_i16((int32_t)(yaw * 10.0f));
  yaw_rate_x10 = clamp_to_i16((int32_t)(mon->yaw_rate_deg_s * 10.0f));
  left_spd_x100 = clamp_to_i16((int32_t)(mon->left_speed_m_s * 100.0f));
  right_spd_x100 = clamp_to_i16((int32_t)(mon->right_speed_m_s * 100.0f));

  pack_int16_hi_lo(yaw_x10, &out[0], &out[1]);
  pack_int16_hi_lo(yaw_rate_x10, &out[2], &out[3]);
  pack_int16_hi_lo(left_spd_x100, &out[4], &out[5]);
  pack_int16_hi_lo(right_spd_x100, &out[6], &out[7]);
}

/* Upper vehicle monitor TX 0x18FF0330 (test/debug payload)
 * data[0]   : throttle_percent (s8, -100..100)
 * data[1]   : steering_percent (s8, -100..100)
 * data[2]   : left_cmd_percent (s8, -100..100)
 * data[3]   : right_cmd_percent (s8, -100..100)
 * data[4:5] : yaw_rate_deg_s * 10 (s16)
 * data[6:7] : center_distance_m * 100 (s16, cm)
 */
static void pack_upper_vehicle_monitor(const rc_intent_t* rc, const motor_cmd_t* cmd_left, const motor_cmd_t* cmd_right,
                                       const vcu_motion_monitor_t* mon, uint8_t out[8]) {
  int32_t max_rc, max_drv;
  int32_t throttle, steering, left_cmd, right_cmd;
  int8_t throttle_pct, steering_pct, left_pct, right_pct;
  int16_t yaw_rate_x10, center_dist_cm;

  memset(out, 0, 8);
  if (!rc || !cmd_left || !cmd_right || !mon)
    return;

  max_rc = (int32_t)g_rcm_vehicle.max_rc_input;
  max_drv = (int32_t)g_rcm_vehicle.max_driver_input;
  if (max_rc <= 0 || max_drv <= 0)
    return;

  throttle = clamp_i32((int32_t)rc->axis3, -max_rc, max_rc);
  steering = clamp_i32((int32_t)rc->axis1, -max_rc, max_rc);
  left_cmd = clamp_i32((int32_t)cmd_left->rpm_axis1, -max_drv, max_drv);
  right_cmd = clamp_i32((int32_t)cmd_right->rpm_axis1, -max_drv, max_drv);

  throttle_pct = (int8_t)clamp_i32((throttle * 100) / max_rc, -100, 100);
  steering_pct = (int8_t)clamp_i32((steering * 100) / max_rc, -100, 100);
  left_pct = (int8_t)clamp_i32((left_cmd * 100) / max_drv, -100, 100);
  right_pct = (int8_t)clamp_i32((right_cmd * 100) / max_drv, -100, 100);

  {
    float mon_yaw_rate = mon->imu_yaw_rate_valid ? mon->imu_yaw_rate_deg_s : mon->yaw_rate_deg_s;
    yaw_rate_x10 = clamp_to_i16((int32_t)(mon_yaw_rate * 10.0f));
  }
  center_dist_cm = clamp_to_i16((int32_t)(mon->center_distance_m * 100.0f));

  out[0] = (uint8_t)throttle_pct;
  out[1] = (uint8_t)steering_pct;
  out[2] = (uint8_t)left_pct;
  out[3] = (uint8_t)right_pct;
  pack_int16_hi_lo(yaw_rate_x10, &out[4], &out[5]);
  pack_int16_hi_lo(center_dist_cm, &out[6], &out[7]);
}

/* Upper weed status TX 0x18FF0320
 * data[0] : status_flags
 * data[1] : error_code
 * data[2] : current_raw
 * data[3] : input_state
 * data[4] : meta bits (bit0 valid, bit1 timeout, bit2 pre_sent)
 * data[5] : target_mm (0..255)
 * data[6] : actual_pos_mm (0..255)
 * data[7] : speed_mm_s (0..255)
 */
static void pack_upper_weed_status(const weed_actuator_status_t* ws, uint16_t target_mm, bool pre_sent, bool timeout,
                                   uint8_t out[8]) {
  uint8_t meta = 0;
  uint16_t pos_mm = 0;
  uint16_t speed_mm_s = 0;

  memset(out, 0, 8);
  if (!ws)
    return;

  pos_mm = ws->position_x10_mm / 10u;
  speed_mm_s = ws->speed_x10_mm_s / 10u;

  if (ws->valid)
    meta |= (1u << 0);
  if (timeout)
    meta |= (1u << 1);
  if (pre_sent)
    meta |= (1u << 2);

  out[0] = ws->status_flags;
  out[1] = ws->error_code;
  out[2] = ws->current_raw;
  out[3] = ws->input_state;
  out[4] = meta;
  out[5] = clamp_u16_to_u8(target_mm);
  out[6] = clamp_u16_to_u8(pos_mm);
  out[7] = clamp_u16_to_u8(speed_mm_s);
}

/* Upper blade status TX 0x18FF0330
 * data[0] : blade left fault_bits
 * data[1] : blade right fault_bits
 * data[2] : blade cmd rpm (0..255, saturated)
 * data[3] : control source bitmask (bit0 RC, bit1 UPPER_AUTO, bit2 STOP)
 * data[4] : blade left rpm_axis1 / 10 (int8)
 * data[5] : blade right rpm_axis1 / 10 (int8)
 * data[6] : blade left valid/fresh (bit0 valid, bit1 fresh)
 * data[7] : blade right valid/fresh (bit0 valid, bit1 fresh)
 */
static void pack_upper_blade_status(const blade_status_t* left, const blade_status_t* right, const blade_cmd_t* cmd,
                                    const upper_status_t* st, rt_tick_t now, uint8_t out[8]) {
  int32_t left_rpm_x10 = 0;
  int32_t right_rpm_x10 = 0;
  uint8_t src_mask = 0u;
  uint8_t left_meta = 0u;
  uint8_t right_meta = 0u;

  memset(out, 0, 8);
  if (!left || !right || !cmd || !st)
    return;

  if (st->control_src == FSM_CTRL_SRC_RC)
    src_mask |= (1u << 0);
  if (st->control_src == FSM_CTRL_SRC_UPPER_AUTO)
    src_mask |= (1u << 1);
  if (st->control_src == FSM_CTRL_SRC_STOP)
    src_mask |= (1u << 2);

  if (left->valid)
    left_meta |= (1u << 0);
  if (right->valid)
    right_meta |= (1u << 0);
  if (left->valid && is_fresh_tick(now, left->ts, MOTOR_TIMEOUT_MS))
    left_meta |= (1u << 1);
  if (right->valid && is_fresh_tick(now, right->ts, MOTOR_TIMEOUT_MS))
    right_meta |= (1u << 1);

  left_rpm_x10 = clamp_i32((int32_t)left->rpm_axis1 / 10, -127, 127);
  right_rpm_x10 = clamp_i32((int32_t)right->rpm_axis1 / 10, -127, 127);

  out[0] = left->fault_bits;
  out[1] = right->fault_bits;
  out[2] = clamp_u16_to_u8(cmd->rpm_cmd);
  out[3] = src_mask;
  out[4] = (uint8_t)((int8_t)left_rpm_x10);
  out[5] = (uint8_t)((int8_t)right_rpm_x10);
  out[6] = left_meta;
  out[7] = right_meta;
}

/* Weed actuator pre-command (1-shot before position control start).
 * Requested payload spec: 03 FB 00 00 00 00 00.
 * TX uses DLC=8, so the last byte is padded with 00.
 */
static void pack_weed_actuator_pre_cmd(uint8_t out[8]) {
  memset(out, 0, 8);
  out[0] = 0x03;
  out[1] = 0xFB;
}

static void pack_weed_actuator_pre_in_cmd(uint8_t out[8]) {
  memset(out, 0, 8);
  out[0] = 0x02;
  out[1] = 0xFB;
	out[2] = 0xFB;
	out[3] = 0xFB;
	out[4] = 0xFB;
	out[5] = 0xFB;
	out[6] = 0xFF;
	out[7] = 0xFF;
	
}
static void pack_weed_actuator_pre_out_cmd(uint8_t out[8]) {
  memset(out, 0, 8);
	out[0] = 0x01;
  out[1] = 0xFB;
	out[2] = 0xFB;
	out[3] = 0xFB;
	out[4] = 0xFB;
	out[5] = 0xFB;
	out[6] = 0xFF;
	out[7] = 0xFF;
}

static void pack_weed_actuator_dir_cmd(bool move_down, uint8_t out[8]) {
  if (move_down) {
    pack_weed_actuator_pre_out_cmd(out); /* down: per_out */
  } else {
    pack_weed_actuator_pre_in_cmd(out); /* up: per_in */
  }
}
/* Weed actuator position command (250ms periodic):
 * - position value goes to data[0:1] (u16, little-endian)
 * - data[2:3] are fixed 0xFB, 0xFB
 * - special command bytes (0x01/0x02) are not used here
 */
static void pack_weed_actuator_pos_cmd(uint16_t target_mm, uint8_t out[8]) {
  uint16_t pos_raw;
  if (target_mm > WEED_ACT_POS_MAX_MM)
    target_mm = WEED_ACT_POS_MAX_MM;
  pos_raw = (uint16_t)(target_mm * WEED_ACT_POS_SCALE_X10);

  memset(out, 0, 8);
  out[0] = (uint8_t)(pos_raw & 0xFFu);        /* little-endian LSB */
  out[1] = (uint8_t)((pos_raw >> 8) & 0xFFu); /* little-endian MSB */
  out[2] = 0xFBu;
  out[3] = 0xFBu;
  out[4] = 0xFBu;
  out[5] = 0xFBu;
  out[6] = 0xFFu;
  out[7] = 0xFFu;
}

static bool weed_period_elapsed(rt_tick_t now, rt_tick_t* last_tick, uint32_t period_ms) {
  uint32_t dt_ms;
  if (!last_tick)
    return false;
  if (*last_tick == 0) {
    *last_tick = now;
    return true;
  }
  dt_ms = (uint32_t)((now - *last_tick) * 1000u / RT_TICK_PER_SECOND);
  if (dt_ms >= period_ms) {
    *last_tick = now;
    return true;
  }
  return false;
}

static uint32_t tick_to_ms(rt_tick_t now, rt_tick_t start_tick) {
  if (start_tick == 0)
    return 0u;
  return (uint32_t)((now - start_tick) * 1000u / RT_TICK_PER_SECOND);
}

/* Blade command decision:
 * - RC CH6 stage(0/250/500rpm) 기반으로 blade 목표 rpm을 계산
 * - FSM STOP/timeout/fault 조건이면 rpm=0으로 강제
 * - 실제 CAN 송신 주기 제어는 TX thread가 담당(250ms periodic)
 */
static void blade_cmd_step(bool cmd_active, uint16_t blade_rpm_cmd, const upper_status_t* st, blade_cmd_t* out) {
  bool run_allowed;

  if (!st || !out)
    return;

  run_allowed = (st->control_src != FSM_CTRL_SRC_STOP) && (st->stop_reason == FSM_STOP_REASON_NONE);

  out->rpm_cmd = (cmd_active && run_allowed) ? blade_rpm_cmd : 0u;
}

/* Build blade TX pending plan in FSM, TX thread only sends pending frames.
 * Policy: send periodic blade commands only when RC B button(enable) is ON.
 */
static void blade_tx_plan_step(rt_tick_t now, bool blade_tx_enable, uint16_t blade_rpm_cmd, blade_tx_plan_t* plan) {
  static rt_tick_t last_blade_tx_tick = 0;
  if (plan == RT_NULL)
    return;

  if (!blade_tx_enable) {
    plan->left_pending = false;
    plan->right_pending = false;
    last_blade_tx_tick = 0;
    return;
  }

  if (!weed_period_elapsed(now, &last_blade_tx_tick, BLADE_TX_PERIOD_MS))
    return;

  pack_blade_cmd_frame(true, blade_rpm_cmd, plan->left_frame);
  plan->left_dlc = 8;
  plan->left_pending = true;

  pack_blade_cmd_frame(false, blade_rpm_cmd, plan->right_frame);
  plan->right_dlc = 8;
  plan->right_pending = true;
}

/* Blade FSM step:
 * 1) decide blade target rpm from current control context
 * 2) build periodic TX pending plan (enabled only by RC B button)
 */
static void blade_fsm_step(rt_tick_t now, bool blade_tx_enable, bool cmd_active, uint16_t blade_rpm_cmd,
                           const upper_status_t* st, blade_cmd_t* cmd, blade_tx_plan_t* plan) {
  if (!st || !cmd || !plan)
    return;

  blade_cmd_step(cmd_active, blade_rpm_cmd, st, cmd);
  blade_tx_plan_step(now, blade_tx_enable, cmd->rpm_cmd, plan);
}

/* 위치기반 Weed FSM (기존 로직 유지):
 * - 목표 위치 변경 시 pre 명령(03 FB ...) 1회 전송
 * - 250ms 주기로 위치 명령 전송
 * - 실제 위치가 목표 근처에 도달하면 pre를 재무장(re-arm)
 *   -> 다음 목표 변경 시 다시 pre부터 시작
 */
static void weed_fsm_step_position_based(rt_tick_t now, bool actuator_requested, uint16_t weed_target_mm,
                                         const upper_status_t* st, const weed_actuator_status_t* ws,
                                         weed_tx_plan_t* plan) {
  bool run_allowed;
  bool weed_rx_fresh;
  bool weed_pos_reached = false;
  bool weed_pos_in_hold_db = false;
  bool pos_due;
  uint16_t weed_pos_mm = 0;
  uint16_t target_diff_mm = 0;

  if (!st || !ws || !plan)
    return;

  run_allowed = (st->control_src != FSM_CTRL_SRC_STOP) && (st->stop_reason == FSM_STOP_REASON_NONE);
  weed_rx_fresh = ws->valid && is_fresh_tick(now, ws->ts, WEED_ACTUATOR_TIMEOUT_MS);
  plan->target_mm = weed_target_mm;
  plan->pre_sent = g_weed_fsm_ctx.pre_sent;
  plan->rx_timeout = !weed_rx_fresh;

  if (weed_rx_fresh) {
    uint16_t diff_mm;
    weed_pos_mm = (uint16_t)(ws->position_x10_mm / 10u);
    diff_mm = (weed_pos_mm >= weed_target_mm) ? (weed_pos_mm - weed_target_mm) : (weed_target_mm - weed_pos_mm);
    weed_pos_reached = (diff_mm <= WEED_ACTUATOR_POS_TOL_MM);
    weed_pos_in_hold_db = (diff_mm <= WEED_POS_HOLD_DB_MM);
  }

  if (!actuator_requested || !run_allowed) {
    g_weed_fsm_ctx.pre_sent = false;
    g_weed_fsm_ctx.target_latched = false;
    g_weed_fsm_ctx.seq_state = WEED_SEQ_IDLE;
    g_weed_fsm_ctx.pre_guard_start_tick = 0;
    g_weed_fsm_ctx.move_window_start_tick = 0;
    g_weed_fsm_ctx.last_pos_tx_tick = 0;
    plan->pre_pending = false;
    plan->dir_pending = false;
    plan->pos_pending = false;
    plan->pre_sent = false;
    return;
  }

  if (!g_weed_fsm_ctx.target_latched) {
    g_weed_fsm_ctx.target_active_mm = weed_target_mm;
    g_weed_fsm_ctx.move_is_down = (weed_target_mm > WEED_POS_UP_MM);
    g_weed_fsm_ctx.target_latched = true;
    g_weed_fsm_ctx.pre_sent = false;
  } else {
    target_diff_mm = (weed_target_mm >= g_weed_fsm_ctx.target_active_mm)
                         ? (uint16_t)(weed_target_mm - g_weed_fsm_ctx.target_active_mm)
                         : (uint16_t)(g_weed_fsm_ctx.target_active_mm - weed_target_mm);
  }

  if (g_weed_fsm_ctx.target_latched && (target_diff_mm >= WEED_TARGET_CHANGE_DB_MM)) {
    g_weed_fsm_ctx.move_is_down = (weed_target_mm > g_weed_fsm_ctx.target_active_mm);
    g_weed_fsm_ctx.target_active_mm = weed_target_mm;
    g_weed_fsm_ctx.pre_sent = false;
  }

  if (weed_pos_reached) {
    /* Re-arm for next command cycle when actual position reaches current target. */
    g_weed_fsm_ctx.pre_sent = false;
  }

  // Requested payload spec: 03 FB 00 00 00 00 00
  if (!g_weed_fsm_ctx.pre_sent && !weed_pos_reached) {
    if (!plan->pre_pending) {
      /* Mark pending so TX thread can send this event frame asynchronously. */
      pack_weed_actuator_pre_cmd(plan->pre_frame);
      plan->pre_dlc = 8u;
      plan->pre_pending = true;
    }
    g_weed_fsm_ctx.pre_sent = true;
  }

  pos_due = weed_period_elapsed(now, &g_weed_fsm_ctx.last_pos_tx_tick, WEED_ACTUATOR_TX_PERIOD_MS);
  if (weed_pos_in_hold_db) {
    /* 실제 위치가 목표 deadband 내면 불필요한 position TX를 억제한다. */
    plan->pos_pending = false;
  } else if (pos_due && !plan->pos_pending) {
    /* Keep latest periodic position frame pending until TX thread consumes it. */
    pack_weed_actuator_pos_cmd(weed_target_mm, plan->pos_frame);
    plan->pos_dlc = 8u;
    plan->pos_pending = true;
  }

  plan->pre_sent = g_weed_fsm_ctx.pre_sent;
  plan->rx_timeout = !weed_rx_fresh;
}

/* 시간기반 Weed FSM (신규 테스트용):
 * - RC 토글(상/중/하) 목표가 바뀌면 트리거 발생
 * - 트리거 시 시퀀스: pre(03 FB) -> direction(01/02 FB) -> pre(03 FB) -> position periodic
 * - 두 pre/direction은 pending clear(송신 완료) 이후 다음 단계로 진행
 * - pre2 후 WEED_ACT_PRE_GUARD_MS 대기
 * - 이후 WEED_ACTUATOR_TX_PERIOD_MS(250ms) 주기로 위치 명령 전송
 * - 트리거 시점부터 WEED_ACT_MOVE_WINDOW_MS(기본 5초) 동안만 송신
 * - 5초 내 새 트리거가 들어오면 윈도우를 다시 시작
 * - 위치 피드백은 모니터링/timeout 판단만 사용, 송신 중단 판단은 시간 기준
 */
static void weed_fsm_step_time_based(rt_tick_t now, bool actuator_requested, uint16_t weed_target_mm,
                                     const upper_status_t* st, const weed_actuator_status_t* ws, weed_tx_plan_t* plan) {
  bool run_allowed;
  bool weed_rx_fresh;
  bool target_changed = false;
  bool move_window_active = false;
  bool pre_guard_done = false;
  bool can_send_next = false;
  bool weed_pos_in_hold_db = false;
	bool stream_flag = false;
  uint16_t target_diff_mm = 0;
  uint16_t weed_pos_mm = 0;

  if (!st || !ws || !plan)
    return;

  run_allowed = (st->control_src != FSM_CTRL_SRC_STOP) && (st->stop_reason == FSM_STOP_REASON_NONE);
  weed_rx_fresh = ws->valid && is_fresh_tick(now, ws->ts, WEED_ACTUATOR_TIMEOUT_MS);
  if (weed_rx_fresh) {
    uint16_t diff_mm;
    weed_pos_mm = (uint16_t)(ws->position_x10_mm / 10u);
    diff_mm = (weed_pos_mm >= weed_target_mm) ? (weed_pos_mm - weed_target_mm) : (weed_target_mm - weed_pos_mm);
    weed_pos_in_hold_db = (diff_mm <= WEED_POS_HOLD_DB_MM);
  }
  plan->target_mm = weed_target_mm;
  plan->pre_sent = g_weed_fsm_ctx.pre_sent;
  plan->rx_timeout = !weed_rx_fresh;

  if (!actuator_requested || !run_allowed) {
    g_weed_fsm_ctx.pre_sent = false;
    g_weed_fsm_ctx.target_latched = false;
    g_weed_fsm_ctx.seq_state = WEED_SEQ_IDLE;
    g_weed_fsm_ctx.pre_guard_start_tick = 0;
    g_weed_fsm_ctx.move_window_start_tick = 0;
    g_weed_fsm_ctx.last_pos_tx_tick = 0;
    plan->pre_pending = false;
    plan->dir_pending = false;
    plan->pos_pending = false;
    plan->pre_sent = false;
    return;
  }

  if (!g_weed_fsm_ctx.target_latched) {
    target_changed = true;
		stream_flag = false;
    g_weed_fsm_ctx.target_latched = true;
    g_weed_fsm_ctx.move_is_down = (weed_target_mm > WEED_POS_UP_MM);
    g_weed_fsm_ctx.target_active_mm = weed_target_mm;
  } else {
    target_diff_mm = (weed_target_mm >= g_weed_fsm_ctx.target_active_mm)
                         ? (uint16_t)(weed_target_mm - g_weed_fsm_ctx.target_active_mm)
                         : (uint16_t)(g_weed_fsm_ctx.target_active_mm - weed_target_mm);
  }

  if (g_weed_fsm_ctx.target_latched && (target_diff_mm >= WEED_TARGET_CHANGE_DB_MM)) {
    target_changed = true;
		stream_flag = false;
    g_weed_fsm_ctx.move_is_down = (weed_target_mm > g_weed_fsm_ctx.target_active_mm);
    g_weed_fsm_ctx.target_active_mm = weed_target_mm;
  }

  if (target_changed) {
    g_weed_fsm_ctx.pre_sent = false;
    g_weed_fsm_ctx.seq_state = WEED_SEQ_WAIT_PRE1_SENT;
    g_weed_fsm_ctx.pre_guard_start_tick = 0;
    g_weed_fsm_ctx.move_window_start_tick = now;
    g_weed_fsm_ctx.last_pos_tx_tick = 0;
    plan->dir_pending = false;
		stream_flag = false;
  }

  if (g_weed_fsm_ctx.move_window_start_tick != 0) {
    move_window_active = (tick_to_ms(now, g_weed_fsm_ctx.move_window_start_tick) < WEED_ACT_MOVE_WINDOW_MS);
  }

  if (!move_window_active) {
    plan->pre_pending = false;
    plan->dir_pending = false;
    plan->pos_pending = false;
    plan->pre_sent = false;
    g_weed_fsm_ctx.pre_sent = false;
    g_weed_fsm_ctx.seq_state = WEED_SEQ_IDLE;
    g_weed_fsm_ctx.pre_guard_start_tick = 0;
		stream_flag = false;
    return;
  }

  can_send_next = (!plan->pre_pending && !plan->dir_pending && !plan->pos_pending);

  switch (g_weed_fsm_ctx.seq_state) {
  case WEED_SEQ_WAIT_PRE1_SENT:
    if (can_send_next) {
      /* step1: pre command */
      pack_weed_actuator_pre_cmd(plan->pre_frame);
      plan->pre_dlc = 8u;
      plan->pre_pending = true;
      g_weed_fsm_ctx.pre_sent = true;
      plan->pre_sent = true;
			g_weed_fsm_ctx.pre_guard_start_tick = now;
      //g_weed_fsm_ctx.seq_state = WEED_SEQ_WAIT_DIR_SENT;
			g_weed_fsm_ctx.seq_state = WEED_SEQ_POS_STREAM;
    
    }
    return;

  case WEED_SEQ_WAIT_DIR_SENT:
    if (can_send_next) {
      /* step2: direction command (per_out/per_in) */
      pack_weed_actuator_dir_cmd(g_weed_fsm_ctx.move_is_down, plan->dir_frame);
      plan->dir_dlc = 8u;
      plan->dir_pending = true;
      if(!stream_flag) 
				g_weed_fsm_ctx.seq_state = WEED_SEQ_WAIT_PRE2_SENT;
			else
				g_weed_fsm_ctx.seq_state = WEED_SEQ_POS_STREAM;

			//g_weed_fsm_ctx.pre_guard_start_tick = now;
			//g_weed_fsm_ctx.seq_state = WEED_SEQ_WAIT_DIR_SENT;
    
    }
    return;

  case WEED_SEQ_WAIT_PRE2_SENT:
    if (can_send_next) {
      /* step3: pre command again */
      pack_weed_actuator_pre_cmd(plan->pre_frame);
      plan->pre_dlc = 8u;
      plan->pre_pending = true;
      g_weed_fsm_ctx.pre_guard_start_tick = now;
      g_weed_fsm_ctx.seq_state = WEED_SEQ_POS_STREAM;
    }
    return;

  case WEED_SEQ_POS_STREAM:
		stream_flag = true;
    break;

  case WEED_SEQ_IDLE:
  default:
    return;
  }

  if (g_weed_fsm_ctx.pre_guard_start_tick != 0) {
    pre_guard_done = (tick_to_ms(now, g_weed_fsm_ctx.pre_guard_start_tick) >= WEED_ACT_PRE_GUARD_MS);
  }
  if (!pre_guard_done)
    return;

  if (weed_pos_in_hold_db) {
    /* 목표 근처(hold deadband)에서는 position 주기 TX를 멈춘다. */
    plan->pos_pending = false;
    return;
  }

  if (weed_period_elapsed(now, &g_weed_fsm_ctx.last_pos_tx_tick, WEED_ACTUATOR_TX_PERIOD_MS) && !plan->pos_pending) {
    /* During move-window, periodic position frame stays pending until TX consumes it. */
    pack_weed_actuator_pos_cmd(weed_target_mm, plan->pos_frame);
    plan->pos_dlc = 8u;
    plan->pos_pending = true;
		//g_weed_fsm_ctx.seq_state = WEED_SEQ_WAIT_DIR_SENT;
		g_weed_fsm_ctx.seq_state = WEED_SEQ_WAIT_PRE2_SENT;
  }
}

/* Weed FSM entry:
 * WEED_FSM_MODE로 위치기반/시간기반 중 하나를 선택한다.
 * 기본값은 위치기반(POSITION_BASED)이라 기존 동작이 유지된다.
 */
static void weed_fsm_step(rt_tick_t now, bool actuator_requested, uint16_t weed_target_mm, const upper_status_t* st,
                          const weed_actuator_status_t* ws, weed_tx_plan_t* plan) {
#if (WEED_FSM_MODE == WEED_FSM_MODE_TIME_BASED)
  weed_fsm_step_time_based(now, actuator_requested, weed_target_mm, st, ws, plan);
#else
  weed_fsm_step_position_based(now, actuator_requested, weed_target_mm, st, ws, plan);
#endif
}

static bool select_upper_weed_target(const upper_weed_cmd_t* cmd, uint16_t* target_mm) {
  uint16_t selected = WEED_POS_UP_MM;

  if (!cmd || !cmd->valid || !target_mm)
    return false;

  if (cmd->target_position_mm != 0u)
    selected = cmd->target_position_mm;
  else
    selected = map_upper_weed_stage_to_mm(cmd->stage);

  *target_mm = (uint16_t)clamp_i32((int32_t)selected, 0, WEED_ACT_POS_MAX_MM);

  /* SET_TARGET only stores the target in the latest cache.
   * MOVE_TO_TARGET is the explicit actuator motion request.
   */
  return (cmd->command_type == UPPER_WEED_CMD_MOVE_TO_TARGET);
}

static bool select_upper_blade_rpm(const upper_blade_cmd_t* cmd, uint16_t* rpm_cmd) {
  uint32_t rpm_avg = 0u;

  if (!cmd || !cmd->valid || !rpm_cmd)
    return false;

  rpm_avg = ((uint32_t)cmd->left_rpm_cmd + (uint32_t)cmd->right_rpm_cmd) / 2u;
  *rpm_cmd = (uint16_t)clamp_i32((int32_t)rpm_avg, 0, BLADE_RPM_CMD_MAX);

  /* Current blade TX path uses one synchronized RPM command for left/right.
   * SET_RPM stores the selected value; RUN makes the command active.
   */
  return (cmd->command_type == UPPER_BLADE_CMD_RUN);
}

/* ===================== Threads ===================== */

/* 1) SBUS thread: update rc_intent */

static void sbus_thread_entry(void* parameter) {
  (void)parameter;
  uint8_t cmd[8] = { 0 };
  uint8_t rpm_v[2] = { 0 };
  uint8_t frame[25];
  SBUS_CH_DATA ch;
  sbus_ma_filter_t axis1_ma;
  sbus_ma_filter_t axis2_ma;
  sbus_ma_filter_t axis3_ma;
  sbus_ma_filter_t axis4_ma;

  sbus_ma_reset(&axis1_ma);
  sbus_ma_reset(&axis2_ma);
  sbus_ma_reset(&axis3_ma);
  sbus_ma_reset(&axis4_ma);

  for (;;) {
    bool failsafe = false, lost = false;

    /* [STEP 1] Get one SBUS frame (25B) and decode channels. */
    if (!sbus_get_frame_25b(frame)) {
      rt_thread_delay(5);
      continue;
    }

    if (!sbus_decode_25b_to_channels(frame, &ch, &failsafe, &lost))
      continue;
#if 1
    sbus_data_raw_a.CH1 = ch.CH1;
    sbus_data_raw_a.CH2 = ch.CH2;
    sbus_data_raw_a.CH3 = ch.CH3;
    sbus_data_raw_a.CH4 = ch.CH4;
    sbus_data_raw_a.CH5 = ch.CH5;
    sbus_data_raw_a.CH6 = ch.CH6;
    sbus_data_raw_a.CH7 = ch.CH7;
    sbus_data_raw_a.CH8 = ch.CH8;
    sbus_data_raw_a.CH9 = ch.CH9;
    sbus_data_raw_a.CH10 = ch.CH10;
    sbus_data_raw_a.CH11 = ch.CH11;
    sbus_data_raw_a.CH12 = ch.CH12;
    sbus_data_raw_a.CH13 = ch.CH13;
    sbus_data_raw_a.CH14 = ch.CH14;
    sbus_data_raw_a.CH15 = ch.CH15;
    sbus_data_raw_a.CH16 = ch.CH16;

#endif
    rc_intent_t rc;
    memset(&rc, 0, sizeof(rc));
    rc.ts = now_tick();
    rc.valid = (!failsafe && !lost);
    rc.failsafe = failsafe;

    /* [STEP 2] On signal loss/failsafe, reset filter history to avoid stale averaging. */
    if (failsafe || lost) {
      sbus_ma_reset(&axis1_ma);
      sbus_ma_reset(&axis2_ma);
      sbus_ma_reset(&axis3_ma);
      sbus_ma_reset(&axis4_ma);
    }

    /* TODO: map channels properly */
    rc.weed_target_mm = map_ch5_to_weed_target_mm(ch.CH5);
    rc.cultivator_down = (rc.weed_target_mm > 0u);
    rc.blade_rpm_cmd = map_ch6_to_blade_rpm(ch.CH6);
    rc.cultivator_on = (rc.blade_rpm_cmd > 0u);
    rc.rc_emergency_stop = (ch.CH8 > 1000);
    rc.rc_enable = (ch.CH9 > 1000);
    /* RC C button (CH10):
     * true  -> stable mode (high-speed steering clamp ON, DEX OFF)
     * false -> agile mode  (high-speed steering clamp OFF, DEX ON)
     */
    rc.rc_drive_mode = (ch.CH10 > 1000);
    rc.rc_remote_automation = (ch.CH11 > 1000);

    /* [STEP 3] Convert SBUS raw -> control command and then smooth by MA(10). */
    rc.axis1 = sbus_convert_to_control(ch.CH1, rpm_v); /* CH1 raw */
    rc.axis2 = sbus_convert_to_control(ch.CH2, rpm_v); /* CH2 raw */
    rc.axis3 = sbus_convert_to_control(ch.CH3, rpm_v); /* CH3 raw */
    rc.axis4 = sbus_convert_to_control(ch.CH4, rpm_v); /* CH4 raw */

    rc.axis1 = sbus_ma_update(&axis1_ma, rc.axis1); /* CH1 filtered */
    rc.axis2 = sbus_ma_update(&axis2_ma, rc.axis2); /* CH2 filtered */
    rc.axis3 = sbus_ma_update(&axis3_ma, rc.axis3); /* CH3 filtered */
    rc.axis4 = sbus_ma_update(&axis4_ma, rc.axis4); /* CH4 filtered */

    /* [STEP 4] Differential-drive mixing (CH3 throttle, CH1 steering):
     * CH3 = throttle (forward/backward)
     * CH1 = steering (left/right)
     *
     * Note:
     * - mix_rc_to_tracks() includes high-speed steering clamp and turn-shaping
     *   (inner/outer ratio control), then outputs left/right driver commands.
     */

    // vcu_diff_drive_mix(rc.axis3, rc.axis1, &rc.left_rpm_value, &rc.right_rpm_value);

    rc_input_t rc_mix_in;
    motor_output_t rc_mix_out;
    memset(&rc_mix_state, 0, sizeof(rc_mix_state));
    rc_mix_in.throttle = (float)rc.axis3;
    rc_mix_in.steering = (float)rc.axis1;
    rc_mix_out = mix_rc_to_tracks(&rc_mix_in, rc.rc_drive_mode, &g_rcm_vehicle, &g_rcm_tune, &rc_mix_state);
    rc.left_rpm_value = rc_mix_out.left_input;
    rc.right_rpm_value = rc_mix_out.right_input;
    (void)rc_mix_state; /* available for debug/logging if needed */

    // rc.axis1 = (int16_t)((int32_t)ch[1] - 992); /* CH2 */
    // rc.axis2 = (int16_t)((int32_t)ch[3] - 992); /* CH4 */

    rt_mutex_take(g_lock, RT_WAITING_FOREVER);
    g_latest.rc = rc;
    rt_mutex_release(g_lock);
  }
}

/* 2) FSM thread: arbitration -> motor_cmd + upper_status */
static void fsm_thread_entry(void* parameter) {
  (void)parameter;

  rc_intent_t rc;
  upper_intent_t upper;
  upper_intent_drive_t upper_drive;
  upper_intent_auto_t upper_auto;
  upper_weed_cmd_t upper_weed;
  upper_blade_cmd_t upper_blade;
  motor_status_t motor_left_st;
  motor_status_t motor_right_st;
  blade_cmd_t blade_cmd;
  blade_tx_plan_t blade_plan;
  weed_actuator_status_t weed_st;
  weed_tx_plan_t weed_plan;
  vcu_motion_monitor_t motion_monitor;

  for (;;) {
    rt_thread_delay(FSM_PERIOD_MS);
    rt_tick_t now = now_tick();

    rt_mutex_take(g_lock, RT_WAITING_FOREVER);
    rc = g_latest.rc;
    upper = g_latest.upper_cmd_config;
    upper_drive = g_latest.upper_cmd_drive;
    upper_auto = g_latest.upper_cmd_auto;
    upper_weed = g_latest.upper_cmd_weed;
    upper_blade = g_latest.upper_cmd_blade;
    /* motor driver status */
    motor_left_st = g_latest.motor_left;
    motor_right_st = g_latest.motor_right;
    blade_cmd = g_latest.blade_cmd;
    blade_plan = g_latest.blade_tx_plan;
    weed_st = g_latest.weed_actuator;
    weed_plan = g_latest.weed_tx_plan;
    motion_monitor = g_latest.motion_monitor;
    rt_mutex_release(g_lock);

    bool rc_ok = rc.valid && is_fresh_tick(now, rc.ts, SBUS_TIMEOUT_MS);
    bool upper_cfg_valid = upper.valid; /* config timeout is not used as a hard validity gate */
    bool upper_drive_ok = upper_drive.valid && is_fresh_tick(now, upper_drive.ts, UPPER_DRIVE_TIMEOUT_MS);
    bool upper_auto_ok = upper_auto.valid && is_fresh_tick(now, upper_auto.ts, UPPER_DRIVE_TIMEOUT_MS);

    /* motor driver status check */
    bool motor_left_ok = motor_left_st.valid && is_fresh_tick(now, motor_left_st.ts, MOTOR_TIMEOUT_MS) &&
                         (motor_left_st.fault_bits == 0);
    bool motor_right_ok = motor_right_st.valid && is_fresh_tick(now, motor_right_st.ts, MOTOR_TIMEOUT_MS) &&
                          (motor_right_st.fault_bits == 0);

    bool rc_timeout = (rc.ts != 0) && !rc_ok;
    bool upper_drive_timeout = (upper_drive.ts != 0) && !upper_drive_ok;
    bool upper_auto_timeout = (upper_auto.ts != 0) && !upper_auto_ok;
    bool motor_left_timeout = (motor_left_st.ts != 0) && !is_fresh_tick(now, motor_left_st.ts, MOTOR_TIMEOUT_MS);
    bool motor_right_timeout = (motor_right_st.ts != 0) && !is_fresh_tick(now, motor_right_st.ts, MOTOR_TIMEOUT_MS);

    bool upper_force_stop = upper.upper_force_stop;
    bool rc_emg = rc.rc_emergency_stop;
    bool rc_active = (rc_ok && rc.rc_enable);
    bool force_upper = (upper.upper_force_active == 1);
    bool upper_auto_ready =
        is_upper_auto_handover_ready(rc_active, rc.rc_remote_automation, upper_cfg_valid, (upper.automation == 1));
    bool weed_upper_active = upper_auto_ready;
    uint16_t upper_weed_target_mm = rc.weed_target_mm;
    uint16_t upper_blade_rpm_cmd = rc.blade_rpm_cmd;
    bool upper_weed_cmd_active = select_upper_weed_target(&upper_weed, &upper_weed_target_mm);
    bool upper_blade_cmd_active = select_upper_blade_rpm(&upper_blade, &upper_blade_rpm_cmd);
    bool weed_cmd_active = weed_upper_active ? upper_weed_cmd_active : rc_active;
    bool blade_cmd_active = weed_upper_active ? upper_blade_cmd_active : rc_active;
    uint16_t weed_target_selected = weed_upper_active ? upper_weed_target_mm : rc.weed_target_mm;
    uint16_t blade_rpm_selected = weed_upper_active ? upper_blade_rpm_cmd : rc.blade_rpm_cmd;

    /* Left motor driver cmd */
    motor_cmd_t out_cmd_left;
    memset(&out_cmd_left, 0, sizeof(out_cmd_left));
    out_cmd_left.ts = now;

    /* Right motor driver cmd */
    motor_cmd_t out_cmd_right;
    memset(&out_cmd_right, 0, sizeof(out_cmd_right));
    out_cmd_right.ts = now;

    /* upper status */
    upper_status_t out_st;
    memset(&out_st, 0, sizeof(out_st));
    out_st.ts = now;
    out_st.timeout_detail_code = TO_NONE;

    out_st.rc_status_mask = 0;
    if (rc.rc_enable)
      out_st.rc_status_mask |= RC_ST_ENABLE;
    if (rc.rc_emergency_stop)
      out_st.rc_status_mask |= RC_ST_EMERGENCY_STOP;
    if (rc.failsafe)
      out_st.rc_status_mask |= RC_ST_FAILSAFE;
    if (rc_ok)
      out_st.rc_status_mask |= RC_ST_FRESH;
    if (rc.cultivator_down)
      out_st.rc_status_mask |= RC_ST_CULTIVATOR_DOWN;
    if (rc.cultivator_on)
      out_st.rc_status_mask |= RC_ST_CULTIVATOR_ON;
    if (rc.rc_remote_automation)
      out_st.rc_status_mask |= RC_ST_REMOTE_AUTOMATION;
    if (rc.rc_drive_mode)
      out_st.rc_status_mask |= RC_ST_DRIVE_MODE;

    /* motor driver status mapping */
    out_st.md_left_fault_msg = (uint8_t)(motor_left_st.fault_bits & 0xFF);
    out_st.md_right_fault_msg = (uint8_t)(motor_right_st.fault_bits & 0xFF);
    out_st.relay_st = upper.relay_mask;
    out_st.power_supply_value =
        (int16_t)clamp_i32((int32_t)motor_left_st.supply_volt, -rcm_max_driver_i32(), rcm_max_driver_i32());

    /* STOP conditions (highest priority) */
    if (upper_force_stop) {
      // out_cmd.src = FSM_CTRL_SRC_STOP; out_cmd.type = CMD_STOP;
      out_cmd_left.src = FSM_CTRL_SRC_STOP;
      out_cmd_left.type = CMD_STOP;
      out_cmd_right.src = FSM_CTRL_SRC_STOP;
      out_cmd_right.type = CMD_STOP;
      out_st.control_src = FSM_CTRL_SRC_STOP;
      out_st.stop_reason = FSM_STOP_UPPER_FORCE;
    } else if (rc_emg) {
      // out_cmd.src = FSM_CTRL_SRC_STOP; out_cmd.type = CMD_STOP;
      out_cmd_left.src = FSM_CTRL_SRC_STOP;
      out_cmd_left.type = CMD_STOP;
      out_cmd_right.src = FSM_CTRL_SRC_STOP;
      out_cmd_right.type = CMD_STOP;
      out_st.control_src = FSM_CTRL_SRC_STOP;
      out_st.stop_reason = FSM_STOP_RC_EMG;
    } else if (!motor_left_ok) {
      // out_cmd.src = FSM_CTRL_SRC_STOP; out_cmd.type = CMD_STOP;
      out_cmd_left.src = FSM_CTRL_SRC_STOP;
      out_cmd_left.type = CMD_STOP;
      out_cmd_right.src = FSM_CTRL_SRC_STOP;
      out_cmd_right.type = CMD_STOP;
      out_st.control_src = FSM_CTRL_SRC_STOP;
      out_st.stop_reason =
          (motor_left_st.valid && motor_left_st.fault_bits != 0) ? FSM_STOP_MOTOR_FAULT : FSM_STOP_TIMEOUT;
      if (out_st.stop_reason == FSM_STOP_TIMEOUT)
        out_st.timeout_detail_code = TO_MOTOR_LEFT;
    } /* else if (!motor_right_ok) {
       out_cmd_left.src = FSM_CTRL_SRC_STOP;
       out_cmd_left.type = CMD_STOP;
       out_cmd_right.src = FSM_CTRL_SRC_STOP;
       out_cmd_right.type = CMD_STOP;
       out_st.control_src = FSM_CTRL_SRC_STOP;
       out_st.stop_reason =
           (motor_right_st.valid && motor_right_st.fault_bits != 0) ? FSM_STOP_MOTOR_FAULT : FSM_STOP_TIMEOUT;
       if (out_st.stop_reason == FSM_STOP_TIMEOUT)
         out_st.timeout_detail_code = TO_MOTOR_RIGHT;
     }*/
    else {
      /* Not STOP:
       * Priority policy:
       * 1) force_upper -> Upper auto-cmd path (override)
       * 2) RC active + auto handover ready -> Upper auto-cmd path
       * 3) RC active + no auto handover -> RC path
       * 4) otherwise -> timeout stop
       */
      if (force_upper || upper_auto_ready) {
        // rt_kprintf("stop_reason :upper_ok \n");
        out_cmd_left.src = FSM_CTRL_SRC_UPPER_AUTO;
        out_cmd_right.src = FSM_CTRL_SRC_UPPER_AUTO;

        if (!upper_auto_ok) {
          out_cmd_left.type = CMD_STOP;
          out_cmd_left.rpm_axis1 = 0;
          out_cmd_left.rpm_axis2 = 0;
          out_cmd_right.type = CMD_STOP;
          out_cmd_right.rpm_axis1 = 0;
          out_cmd_right.rpm_axis2 = 0;
          out_st.stop_reason = FSM_STOP_TIMEOUT; /* upper auto cmd timeout */
          out_st.timeout_detail_code = TO_UPPER_DRIVE;
        } else {
          int16_t left_cmd = 0;
          int16_t right_cmd = 0;
#if (UPPER_AUTO_MIX_MODE == UPPER_AUTO_MIX_MODE_RC_MIXER)
          mix_upper_auto_cmd_via_rc_mixer(upper_auto.linear_mps_x1000, upper_auto.yaw_rate_deg_s_x10, &left_cmd,
                                          &right_cmd);
#else
          mix_upper_auto_cmd_to_tracks(upper_auto.linear_mps_x1000, upper_auto.yaw_rate_deg_s_x10, &left_cmd,
                                       &right_cmd);
#endif

          out_cmd_left.type = CMD_SETPOINT;
          out_cmd_left.rpm_axis1 = left_cmd;
          out_cmd_left.rpm_axis2 = left_cmd;
          out_cmd_right.type = CMD_SETPOINT;
          out_cmd_right.rpm_axis1 = right_cmd;
          out_cmd_right.rpm_axis2 = right_cmd;
        }
        out_st.control_src = FSM_CTRL_SRC_UPPER_AUTO;
        if (out_st.stop_reason != FSM_STOP_TIMEOUT)
          out_st.stop_reason = FSM_STOP_REASON_NONE;
      } else if (rc_active) {
        // rt_kprintf("stop_reason :rc_ok \n");
        out_cmd_left.src = FSM_CTRL_SRC_RC;
        out_cmd_left.type = CMD_SETPOINT;
        out_cmd_right.src = FSM_CTRL_SRC_RC;
        out_cmd_right.type = CMD_SETPOINT;

        /* Left/right value drives both wheels on each side with same command. */
        out_cmd_left.rpm_axis1 = rc.left_rpm_value;
        out_cmd_left.rpm_axis2 = rc.left_rpm_value;
        out_cmd_right.rpm_axis1 = rc.right_rpm_value;
        out_cmd_right.rpm_axis2 = rc.right_rpm_value;

        out_st.control_src = FSM_CTRL_SRC_RC;
        out_st.stop_reason = FSM_STOP_REASON_NONE;
      } else {

        // rt_kprintf("stop_reason : none \n");
        out_cmd_left.src = FSM_CTRL_SRC_STOP;
        out_cmd_left.type = CMD_STOP;
        out_cmd_right.src = FSM_CTRL_SRC_STOP;
        out_cmd_right.type = CMD_STOP;
        out_st.control_src = FSM_CTRL_SRC_STOP;
        out_st.stop_reason = FSM_STOP_TIMEOUT;
        out_st.timeout_detail_code = make_timeout_detail_code(
            rc_timeout, false, (upper_drive_timeout || upper_auto_timeout), motor_left_timeout, motor_right_timeout);
      }
    }

    /*chcek relay on/off of automation flag */
    /*if both automation on to the automation operation*/
    bool automation_flag = upper_auto_ready;
    uint8_t bit_mask = 0;

    if (automation_flag) {
      OpenCloseIO_Out(1, 1);
      bit_mask = 1 << 1;
      out_st.relay_st |= bit_mask;
    } else {
      OpenCloseIO_Out(1, 0);
      bit_mask = 1 << 0;
      out_st.relay_st |= bit_mask;
    }

    /* Apply same default driver configuration to left/right.
     * Accept upper-supplied config only when enable bits are BOTH_ENABLE.
     */
    /* Driver config is fixed to default bits. */
    out_cmd_left.enable_bit = MOTOR_DRV_DEFAULT_ENABLE_BITS;
    out_cmd_right.enable_bit = MOTOR_DRV_DEFAULT_ENABLE_BITS;

    /* Acceleration can be tuned from upper config payload (0x18FF0210 data[6:7]). */
    out_cmd_left.axis1_accel_bit = upper_cfg_valid ? upper.left_accel_cmd : MOTOR_DRV_DEFAULT_AXIS1_ACC;
    out_cmd_left.axis2_accel_bit = upper_cfg_valid ? upper.left_accel_cmd : MOTOR_DRV_DEFAULT_AXIS2_ACC;
    out_cmd_right.axis1_accel_bit = upper_cfg_valid ? upper.right_accel_cmd : MOTOR_DRV_DEFAULT_AXIS1_ACC;
    out_cmd_right.axis2_accel_bit = upper_cfg_valid ? upper.right_accel_cmd : MOTOR_DRV_DEFAULT_AXIS2_ACC;

    /* Build feedback payloads for upper (100ms TX in can_thread). */
    upper_status_rpm_t out_rpm_st;
    memset(&out_rpm_st, 0, sizeof(out_rpm_st));
    out_rpm_st.ts = now;
    out_rpm_st.driver_left_axis1_rpm =
        (int16_t)clamp_i32((int32_t)motor_left_st.rpm_axis1, -rcm_max_driver_i32(), rcm_max_driver_i32());
    out_rpm_st.driver_left_axis2_rpm =
        (int16_t)clamp_i32((int32_t)motor_left_st.rpm_axis2, -rcm_max_driver_i32(), rcm_max_driver_i32());
    out_rpm_st.driver_right_axis1_rpm =
        (int16_t)clamp_i32((int32_t)motor_right_st.rpm_axis1, -rcm_max_driver_i32(), rcm_max_driver_i32());
    out_rpm_st.driver_right_axis2_rpm =
        (int16_t)clamp_i32((int32_t)motor_right_st.rpm_axis2, -rcm_max_driver_i32(), rcm_max_driver_i32());

    out_st.fsm_mode =
        decide_fsm_mode(rc_active, rc.rc_remote_automation, upper_auto_ready, out_st.control_src, out_st.stop_reason);
    out_st.fsm_status_mask = pack_fsm_status_mask(out_st.fsm_mode, out_st.stop_reason);

    /* Command-based monitoring:
     * Integrate heading/distance from commanded left/right inputs (out_cmd),
     * not from motor feedback RPM.
     */
    update_motion_monitor(&motion_monitor, now, out_cmd_left.rpm_axis1, (int16_t)(-out_cmd_right.rpm_axis1));

    /* Weed FSM step: decide pre/periodic actuator commands and status meta. */
    weed_fsm_step(now, weed_cmd_active, weed_target_selected, &out_st, &weed_st, &weed_plan);
    /* Blade FSM step: rpm decision + periodic pending generation(RC B enable gate). */
    blade_fsm_step(now, rc.rc_enable, blade_cmd_active, blade_rpm_selected, &out_st, &blade_cmd, &blade_plan);

    /*add to registry with cmd & status */
    rt_mutex_take(g_lock, RT_WAITING_FOREVER);
    g_latest.motor_cmd_left = out_cmd_left;
    g_latest.motor_cmd_right = out_cmd_right;
    g_latest.upper_rpm_st = out_rpm_st;
    g_latest.upper_vcu_st = out_st;
    weed_plan.pre_pending = (g_latest.weed_tx_plan.pre_pending || weed_plan.pre_pending);
    weed_plan.dir_pending = (g_latest.weed_tx_plan.dir_pending || weed_plan.dir_pending);
    weed_plan.pos_pending = (g_latest.weed_tx_plan.pos_pending || weed_plan.pos_pending);
    g_latest.weed_tx_plan = weed_plan;
    g_latest.blade_cmd = blade_cmd;
    blade_plan.left_pending = (g_latest.blade_tx_plan.left_pending || blade_plan.left_pending);
    blade_plan.right_pending = (g_latest.blade_tx_plan.right_pending || blade_plan.right_pending);
    g_latest.blade_tx_plan = blade_plan;
    g_latest.motion_monitor = motion_monitor;
    rt_mutex_release(g_lock);
  }
}

/* 3-1) CAN RX thread: parse RX frames as fast as possible */
static void can_rx_thread_entry(void* parameter) {
  (void)parameter;

  for (;;) {
    can_frame_t rx;
    if (rt_mq_recv(g_can_rx_mq, &rx, sizeof(rx), RT_WAITING_FOREVER) == RT_EOK) {
      upper_intent_t up;
      if (decode_upper_cmd(&rx, &up)) {
        rt_mutex_take(g_lock, RT_WAITING_FOREVER);
        g_latest.upper_cmd_config = up;
        rt_mutex_release(g_lock);
        continue;
      }

      upper_intent_drive_t up_drive;
      if (decode_upper_drive_cmd(&rx, &up_drive)) {
        rt_mutex_take(g_lock, RT_WAITING_FOREVER);
        g_latest.upper_cmd_drive = up_drive;
        rt_mutex_release(g_lock);
        continue;
      }

      upper_intent_auto_t up_auto;
      if (decode_upper_auto_cmd(&rx, &up_auto)) {
        rt_mutex_take(g_lock, RT_WAITING_FOREVER);
        g_latest.upper_cmd_auto = up_auto;
        rt_mutex_release(g_lock);
        continue;
      }

      upper_weed_cmd_t up_weed;
      if (decode_upper_weed_actuator_cmd(&rx, &up_weed)) {
        rt_mutex_take(g_lock, RT_WAITING_FOREVER);
        g_latest.upper_cmd_weed = up_weed;
        rt_mutex_release(g_lock);
        continue;
      }

      upper_blade_cmd_t up_blade;
      if (decode_upper_blade_cmd(&rx, &up_blade)) {
        rt_mutex_take(g_lock, RT_WAITING_FOREVER);
        g_latest.upper_cmd_blade = up_blade;
        rt_mutex_release(g_lock);
        continue;
      }

      weed_actuator_status_t ws;
      if (decode_weed_actuator_status(&rx, &ws)) {
        rt_mutex_take(g_lock, RT_WAITING_FOREVER);
        g_latest.weed_actuator = ws;
        rt_mutex_release(g_lock);
        continue;
      }

      /* motor driver left status */
      motor_status_t ms_left;
      if (decode_motor_status(&rx, CANID_MOTOR_STATUS_LEFT_RX, &ms_left)) {
        rt_mutex_take(g_lock, RT_WAITING_FOREVER);
        g_latest.motor_left = ms_left;
        rt_mutex_release(g_lock);
        continue;
      }
      /* motor driver right status */
      motor_status_t ms_right;
      if (decode_motor_status(&rx, CANID_MOTOR_STATUS_RIGHT_RX, &ms_right)) {
        rt_mutex_take(g_lock, RT_WAITING_FOREVER);
        g_latest.motor_right = ms_right;
        rt_mutex_release(g_lock);
        continue;
      }

      /* blade driver left status */
      blade_status_t bs_left;
      if (decode_blade_status(&rx, CANID_BLADE_LEFT_RX, &bs_left)) {
        rt_mutex_take(g_lock, RT_WAITING_FOREVER);
        g_latest.blade_left = bs_left;
        rt_mutex_release(g_lock);
        continue;
      }
      /* blade driver right status */
      blade_status_t bs_right;
      if (decode_blade_status(&rx, CANID_BLADE_RIGHT_RX, &bs_right)) {
        rt_mutex_take(g_lock, RT_WAITING_FOREVER);
        g_latest.blade_right = bs_right;
        rt_mutex_release(g_lock);
        continue;
      }
    }
  }
}

/* 3-2) CAN TX thread: 100ms control TX + 200ms upper status/report TX */
static void can_tx_thread_entry(void* parameter) {
  (void)parameter;
  rt_tick_t last_upper_status_tx_tick = 0;

  for (;;) {
    rt_thread_delay(CAN_TX_PERIOD_MS);
    rt_tick_t now = now_tick();

    motor_cmd_t cmd_left;
    motor_cmd_t cmd_right;
    upper_status_t st;
    upper_status_rpm_t st_rpm;
    vcu_motion_monitor_t mon;
    rc_intent_t rc;
    blade_cmd_t blade_cmd;
    blade_tx_plan_t blade_plan;
    blade_status_t blade_left_st;
    blade_status_t blade_right_st;
    weed_actuator_status_t weed_st;
    weed_tx_plan_t weed_plan;

    rt_mutex_take(g_lock, RT_WAITING_FOREVER);
    cmd_left = g_latest.motor_cmd_left;
    cmd_right = g_latest.motor_cmd_right;
    st = g_latest.upper_vcu_st;
    st_rpm = g_latest.upper_rpm_st;
    mon = g_latest.motion_monitor;
    rc = g_latest.rc;
    blade_cmd = g_latest.blade_cmd;
    blade_plan = g_latest.blade_tx_plan;
    blade_left_st = g_latest.blade_left;
    blade_right_st = g_latest.blade_right;
    weed_st = g_latest.weed_actuator;
    weed_plan = g_latest.weed_tx_plan;
    rt_mutex_release(g_lock);

    uint8_t d0[8], d1[8];
    bool sent_pre = false;
    bool sent_dir = false;
    bool sent_pos = false;
    bool sent_blade_left = false;
    bool sent_blade_right = false;

    /* Actuator uses pending semantics:
     * FSM prepares event frames(pre/dir/pos) and marks pending=true.
     * TX thread consumes and clears pending only after successful send.
     */
    if (weed_plan.pre_pending) {
      sent_pre = can_hw_send_ext(CANID_WEED_ACTUATOR_TX, weed_plan.pre_frame, weed_plan.pre_dlc);
    }
    if (weed_plan.dir_pending) {
      sent_dir = can_hw_send_ext(CANID_WEED_ACTUATOR_TX, weed_plan.dir_frame, weed_plan.dir_dlc);
    }
    if (weed_plan.pos_pending) {
      sent_pos = can_hw_send_ext(CANID_WEED_ACTUATOR_TX, weed_plan.pos_frame, weed_plan.pos_dlc);
    }
    if (sent_pre || sent_dir || sent_pos) {
      rt_mutex_take(g_lock, RT_WAITING_FOREVER);
      if (sent_pre)
        g_latest.weed_tx_plan.pre_pending = false;
      if (sent_dir)
        g_latest.weed_tx_plan.dir_pending = false;
      if (sent_pos)
        g_latest.weed_tx_plan.pos_pending = false;
      rt_mutex_release(g_lock);
    }

    /* Blade uses pending semantics (prepared in FSM, consumed in TX). */
    if (blade_plan.left_pending) {
      sent_blade_left = can_hw_send_ext(CANID_BLADE_LEFT_TX, blade_plan.left_frame, blade_plan.left_dlc);
    }
    if (blade_plan.right_pending) {
      sent_blade_right = can_hw_send_ext(CANID_BLADE_RIGHT_TX, blade_plan.right_frame, blade_plan.right_dlc);
    }
    if (sent_blade_left || sent_blade_right) {
      rt_mutex_take(g_lock, RT_WAITING_FOREVER);
      if (sent_blade_left)
        g_latest.blade_tx_plan.left_pending = false;
      if (sent_blade_right)
        g_latest.blade_tx_plan.right_pending = false;
      rt_mutex_release(g_lock);
    }

    /* Driver 1 real operation(run signal)*/
    pack_motor_cmd(&cmd_left, d0);
    (void)can_hw_send_ext(CANID_MOTOR_CMD_DRIVER1_TX, d0, 8);
    /* Driver 2 real operation(run signal)*/
    pack_motor_cmd(&cmd_right, d0);
    (void)can_hw_send_ext(CANID_MOTOR_CMD_DRIVER2_TX, d0, 8);

    if (weed_period_elapsed(now, &last_upper_status_tx_tick, UPPER_STATUS_TX_PERIOD_MS)) {
      /* send vcu status to upper */
      pack_upper_status(&st, d1);
      (void)can_hw_send_ext(CANID_UPPER_STATUS_TX, d1, 8);

      /* send driver left & right feedback rpm data to upper */
      pack_upper_status_rpm(&st_rpm, d1);
      (void)can_hw_send_ext(CANID_UPPER_STATUS_RPM_TX, d1, 8);

      /* send weed actuator status to upper */
      pack_upper_weed_status(&weed_st, weed_plan.target_mm, weed_plan.pre_sent, weed_plan.rx_timeout, d1);
      (void)can_hw_send_ext(CANID_UPPER_WEED_STATUS_TX, d1, 8);

      /* send blade status to upper */
      pack_upper_blade_status(&blade_left_st, &blade_right_st, &blade_cmd, &st, now, d1);
      (void)can_hw_send_ext(CANID_UPPER_BLADE_STATUS_TX, d1, 8);

      /* send vehicle motion status to upper monitor/test ID */
      pack_upper_vehicle_status(&mon, d1);
      (void)can_hw_send_ext(CANID_UPPER_VEHICLE_STATUS_TX, d1, 8);

      /* send vehicle monitor/debug status to upper monitor/test ID */
      pack_upper_vehicle_monitor(&rc, &cmd_left, &cmd_right, &mon, d1);
      (void)can_hw_send_ext(CANID_UPPER_VEHICLE_MON_TX, d1, 8);
    }
  }
}

int vcu_gateway_get_motion_monitor(vcu_motion_monitor_t* out) {
  if (!out || g_lock == RT_NULL)
    return -1;

  rt_mutex_take(g_lock, RT_WAITING_FOREVER);
  *out = g_latest.motion_monitor;
  rt_mutex_release(g_lock);
  return 0;
}

/* ===================== Init ===================== */
int vcu_gateway_init(void) {
  /* mutex */
  g_lock = rt_mutex_create("gwlk", RT_IPC_FLAG_FIFO);
  if (!g_lock)
    return -1;

  /*  init can tx rx mq if not yet */
  //  g_can_rx_mq = rt_mq_create("canrx", sizeof(can_frame_t), CAN_RX_MQ_DEPTH, RT_IPC_FLAG_FIFO);
  //  if (!g_can_rx_mq) return -2;

  if (can_mq_init() != 0) {
    rt_kprintf("can init error! \n");
    return -2;
  }

  /* init SBUS mq if not yet */
  if (sbus_mq_init() != 0) {
    rt_kprintf("sbus init error! \n");
    return -3;
  }

  g_can_rx_mq = can_rx_mq_get();
  if (g_can_rx_mq == RT_NULL) {
    rt_kprintf("can rx queue error! \n");
    return -4;
  }

  /* init shared structs to safe defaults */
  rt_mutex_take(g_lock, RT_WAITING_FOREVER);
  memset(&g_latest, 0, sizeof(g_latest));
  g_latest.motor_cmd_left.type = CMD_STOP;
  g_latest.motor_cmd_left.src = FSM_CTRL_SRC_STOP;
  g_latest.motor_cmd_right.type = CMD_STOP;
  g_latest.motor_cmd_right.src = FSM_CTRL_SRC_STOP;
  rt_mutex_release(g_lock);

  /* threads */
  rt_thread_t th;

  th = rt_thread_create("sbus", sbus_thread_entry, RT_NULL, 1024, 18, 10);
  if (th)
    rt_thread_startup(th);

  th = rt_thread_create("fsm", fsm_thread_entry, RT_NULL, 2048, 16, 10);
  if (th)
    rt_thread_startup(th);

  th = rt_thread_create("canrx", can_rx_thread_entry, RT_NULL, 1024, 15, 10);
  if (th)
    rt_thread_startup(th);

  th = rt_thread_create("cantx", can_tx_thread_entry, RT_NULL, 1024, 17, 10);
  if (th)
    rt_thread_startup(th);

  return 0;
}
