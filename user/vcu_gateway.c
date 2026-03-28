

/*
 * vcu_gateway_rtthread.c
 * RT-Thread 3.0 + MDK-ARM 5.06 skeleton
 *
 * Threads:
 *  - sbus_thread: SBUS -> rc_intent update
 *  - fsm_thread : arbitration -> motor_cmd + upper_status update
 *  - can_thread : CAN RX parse + CAN TX periodic(100ms)
 *
 * CAN:
 *  - Motor status RX: 0x18FF2100
 *  - Upper status TX: 0x18FF0100 (temporary)
 *  - Upper cmd RX ID / Motor cmd TX ID: TODO set
 */

#include <rtthread.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "vcu_gateway.h"
#include "SBUS_AGMO.h"
#include "CAN_AGMO.h"
#include "rc_mixer.h"

typedef struct {
  rt_tick_t ts; /* rt_tick */
  bool valid;
  bool rc_enable;            /* CH10 enable */
  bool rc_emergency_stop;    /* emergency stop */
  bool rc_remote_automation; /* RC D button: remote automation flag */
  bool cultivator_down;
  bool cultivator_on;
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
  uint8_t driver_config_bitmask; /* data[0]: motor driver configuration */
  bool cultivator_down;          /* data[1] */
  bool cultivator_on;            /* data[2] */
  bool upper_force_stop;         /* data[3]: E-stop */
  bool upper_force_active;       /* data[4]: force upper mode */
  uint8_t relay_mask;            /* data[5] */
  bool automation;               /* data[6] */
} upper_intent_t;

typedef struct {
  rt_tick_t ts;
  bool valid;
  int16_t throttle_cmd; /* upper command */
  int16_t steering_cmd; /* upper command */
} upper_intent_drive_t;

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
  fsm_stop_reason_t stop_reason;
  uint8_t rc_status_mask;      /* RC status bit mask */
  uint8_t vcu_fsm_status_mask; /* VCU FSM status bit mask */
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

/* ===================== Shared "latest mails" ===================== */
struct {
  rc_intent_t rc;
  upper_intent_t upper_cmd_config;
  upper_intent_drive_t upper_cmd_drive;
  motor_cmd_t motor_cmd_left;
  motor_cmd_t motor_cmd_right;

  motor_status_t motor_left;
  motor_status_t motor_right;
  upper_status_t upper_vcu_st;
  upper_status_rpm_t upper_rpm_st;
  vcu_motion_monitor_t motion_monitor;
} g_latest;

static rt_mutex_t g_lock = RT_NULL;

/* ===================== CAN RX message queue ===================== */
static rt_mq_t g_can_rx_mq = RT_NULL;
#define CAN_RX_MQ_DEPTH 32

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

int16_t rpm_a;
SBUS_CH_DATA sbus_data_raw_a;

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

static int16_t sbus_to_cmd(int16_t sbus_data) {
  sbus_data = (int16_t)clamp_i32(sbus_data, SBUS_MIN, SBUS_MAX);

  if (sbus_data > (SBUS_CENTER - DEADBAND) && sbus_data < (SBUS_CENTER + DEADBAND)) {
    return 0;
  }
  if (sbus_data >= SBUS_CENTER) {
    int32_t num = (int32_t)(sbus_data - SBUS_CENTER) * CMD_MAX;
    int32_t den = (SBUS_MAX - SBUS_CENTER);
    return (int16_t)((num / den));
  } else {
    int32_t num = (int32_t)(SBUS_CENTER - sbus_data) * (-CMD_MIN);
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

static int16_t sbus_ma_update(sbus_ma_filter_t* f, int16_t sample) {
  if (!f)
    return sample;

  if (f->count < SBUS_FILTER_WINDOW) {
    f->buf[f->idx] = sample;
    f->sum += sample;
    f->idx = (uint8_t)((f->idx + 1u) % SBUS_FILTER_WINDOW);
    f->count++;
    return (int16_t)(f->sum / (int32_t)f->count);
  }

  f->sum -= f->buf[f->idx];
  f->buf[f->idx] = sample;
  f->sum += sample;
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
 *    left/right are clamped to [CMD_MIN, CMD_MAX].
 */

void vcu_diff_drive_mix(int16_t throttle, int16_t steering, int16_t* left, int16_t* right) {
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
    *left = (int16_t)clamp_i32(s, CMD_MIN, CMD_MAX);
    *right = (int16_t)clamp_i32(s, CMD_MIN, CMD_MAX);
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
    *right = (int16_t)clamp_i32(-outer, CMD_MIN, CMD_MAX);
    *left = (int16_t)clamp_i32(inner, CMD_MIN, CMD_MAX);
  } else if (s > 0) {
    *left = (int16_t)clamp_i32(outer, CMD_MIN, CMD_MAX);
    *right = (int16_t)clamp_i32(-inner, CMD_MIN, CMD_MAX);
  } else {
    *left = (int16_t)clamp_i32(t, CMD_MIN, CMD_MAX);
    *right = (int16_t)clamp_i32(-t, CMD_MIN, CMD_MAX);
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
/* Example: decode upper cmd payload (adjust to your protocol) */
static bool decode_upper_cmd(const can_frame_t* rx, upper_intent_t* out) {
  if (rx->ext_id != CANID_UPPER_CMD_RX)
    return false;

  memset(out, 0, sizeof(*out));
  out->ts = now_tick();
  out->valid = true;

  /* Upper -> gateway setting payload (0x18FF0210):
   * data[0]: driver configuration bit mask
   * data[1]: cultivator_down
   * data[2]: cultivator_on
   * data[3]: E-stop
   * data[4]: force upper mode
   * data[5]: relay mask
   * data[6]: automation
   * data[7]: reserved
   */
  out->driver_config_bitmask = (uint8_t)rx->data[0];
  out->cultivator_down = ((rx->data[1] & 0x01u) != 0u);
  out->cultivator_on = ((rx->data[2] & 0x01u) != 0u);
  out->upper_force_stop = ((rx->data[3] & 0x01u) != 0u);
  out->upper_force_active = ((rx->data[4] & 0x01u) != 0u);
  out->relay_mask = (uint8_t)rx->data[5];
  out->automation = ((rx->data[6] & 0x01u) != 0u);

  return true;
}

/* Example: decode upper rpm cmd payload (adjust to your protocol) */
static bool decode_upper_drive_cmd(const can_frame_t* rx, upper_intent_drive_t* out) {
  if (rx->ext_id != CANID_UPPER_CMD_DRIVE_RX)
    return false;
  if (rx->dlc < 4u)
    return false;

  memset(out, 0, sizeof(*out));
  out->ts = now_tick();
  out->valid = false;

  /* Upper -> gateway drive payload (0x18FF0200):
   * data[0:1] : throttle_cmd (int16, signed)
   * data[2:3] : steering_cmd (int16, signed)
   * data[4:7] : reserved
   */
  out->throttle_cmd = (int16_t)((uint16_t)rx->data[0] << 8 | ((uint16_t)rx->data[1]));
  out->steering_cmd = (int16_t)((uint16_t)rx->data[2] << 8 | ((uint16_t)rx->data[3]));
  out->throttle_cmd = (int16_t)clamp_i32((int32_t)out->throttle_cmd, CMD_MIN, CMD_MAX);
  out->steering_cmd = (int16_t)clamp_i32((int32_t)out->steering_cmd, CMD_MIN, CMD_MAX);
  out->valid = true;

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

/* Upper status TX 0x18FF0310 (gateway status) */
static void pack_upper_status(const upper_status_t* st, uint8_t out[8]) {
  memset(out, 0, 8);
  pack_int16_hi_lo(st->power_supply_value, &out[0], &out[1]);
  out[2] = st->md_left_fault_msg;
  out[3] = st->md_right_fault_msg;
  out[4] = st->rc_status_mask;
  out[5] = st->vcu_fsm_status_mask;
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

  yaw_rate_x10 = clamp_to_i16((int32_t)(mon->yaw_rate_deg_s * 10.0f));
  center_dist_cm = clamp_to_i16((int32_t)(mon->center_distance_m * 100.0f));

  out[0] = (uint8_t)throttle_pct;
  out[1] = (uint8_t)steering_pct;
  out[2] = (uint8_t)left_pct;
  out[3] = (uint8_t)right_pct;
  pack_int16_hi_lo(yaw_rate_x10, &out[4], &out[5]);
  pack_int16_hi_lo(center_dist_cm, &out[6], &out[7]);
}

/* ===================== Threads ===================== */

/* 1) SBUS thread: update rc_intent */

calc_state_t rc_mix_state;
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
#if 0	
				sbus_data_raw_a.CH1 = ch[0];
				sbus_data_raw_a.CH2 = ch[1];
				sbus_data_raw_a.CH3 = ch[2];
				sbus_data_raw_a.CH4 = ch[3];
				sbus_data_raw_a.CH5 = ch[4];
				sbus_data_raw_a.CH6 = ch[5];
				sbus_data_raw_a.CH7 = ch[6];
				sbus_data_raw_a.CH8 = ch[7];
				sbus_data_raw_a.CH9 = ch[8];
				sbus_data_raw_a.CH10 = ch[9];
				sbus_data_raw_a.CH11 = ch[10];
				sbus_data_raw_a.CH12 = ch[11];
				sbus_data_raw_a.CH13 = ch[12];
				sbus_data_raw_a.CH14 = ch[13];
				sbus_data_raw_a.CH15 = ch[14];
				sbus_data_raw_a.CH16 = ch[15];
				


				
				make_can_payload_from_sbus(ch[3], cmd);
				
				rpm_v[0] = cmd[2];
				rpm_v[1] = cmd[1];
				rpm_a = (int16_t)((int16_t)rpm_v[0]|((int16_t)rpm_v[1] << 8));
				rt_kprintf("rpm :%d\n", rpm_a);
				for(int i =0; i<8; i++){
					rt_kprintf("cmd :0x%02X, rpm :%d\n", cmd[i], rpm_a);
				}
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
    rc.cultivator_down = (ch.CH5 > 1000);
    rc.cultivator_on = (ch.CH6 > 1000);
    rc.rc_emergency_stop = (ch.CH8 > 1000);
    rc.rc_enable = (ch.CH9 > 1000);
    rc.rc_remote_automation = (ch.CH10 > 1000);

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
    // calc_state_t rc_mix_state;
    motor_output_t rc_mix_out;
    memset(&rc_mix_state, 0, sizeof(rc_mix_state));
    rc_mix_in.throttle = (float)rc.axis3;
    rc_mix_in.steering = (float)rc.axis1;
    rc_mix_out = mix_rc_to_tracks(&rc_mix_in, &g_rcm_vehicle, &g_rcm_tune, &rc_mix_state);
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
  motor_status_t motor_left_st;
  motor_status_t motor_right_st;
  vcu_motion_monitor_t motion_monitor;

  for (;;) {
    rt_thread_delay(FSM_PERIOD_MS);
    rt_tick_t now = now_tick();

    rt_mutex_take(g_lock, RT_WAITING_FOREVER);
    rc = g_latest.rc;
    upper = g_latest.upper_cmd_config;
    upper_drive = g_latest.upper_cmd_drive;
    /* motor driver status */
    motor_left_st = g_latest.motor_left;
    motor_right_st = g_latest.motor_right;
    motion_monitor = g_latest.motion_monitor;
    rt_mutex_release(g_lock);

    bool rc_ok = rc.valid && is_fresh_tick(now, rc.ts, SBUS_TIMEOUT_MS);
    bool upper_ok = upper.valid && is_fresh_tick(now, upper.ts, UPPER_TIMEOUT_MS);
    // bool upper_ok = upper.automation;
    bool upper_drive_ok = upper_drive.valid && is_fresh_tick(now, upper_drive.ts, UPPER_DRIVE_TIMEOUT_MS);

    /* motor driver status check */
    bool motor_left_ok = motor_left_st.valid && is_fresh_tick(now, motor_left_st.ts, MOTOR_TIMEOUT_MS) &&
                         (motor_left_st.fault_bits == 0);
    bool motor_right_ok = motor_right_st.valid && is_fresh_tick(now, motor_right_st.ts, MOTOR_TIMEOUT_MS) &&
                          (motor_right_st.fault_bits == 0);

    bool rc_timeout = (rc.ts != 0) && !rc_ok;
    bool upper_cfg_timeout = (upper.ts != 0) && !upper_ok;
    bool upper_drive_timeout = (upper_drive.ts != 0) && !upper_drive_ok;
    bool motor_left_timeout = (motor_left_st.ts != 0) && !is_fresh_tick(now, motor_left_st.ts, MOTOR_TIMEOUT_MS);
    bool motor_right_timeout = (motor_right_st.ts != 0) && !is_fresh_tick(now, motor_right_st.ts, MOTOR_TIMEOUT_MS);

    bool upper_force_stop = upper.upper_force_stop;
    bool rc_emg = rc.rc_emergency_stop;

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

    /* motor driver status mapping */
    out_st.md_left_fault_msg = (uint8_t)(motor_left_st.fault_bits & 0xFF);
    out_st.md_right_fault_msg = (uint8_t)(motor_right_st.fault_bits & 0xFF);
    out_st.relay_st = upper.relay_mask;
    out_st.power_supply_value = (int16_t)clamp_i32((int32_t)motor_left_st.supply_volt, CMD_MIN, CMD_MAX);

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
       * 1) upper_force_active=1 -> force Upper path even when RC is active
       * 2) else RC active       -> RC path
       * 3) else upper_ok        -> Upper path
       * 4) else                 -> timeout stop
       */
      bool rc_active = (rc_ok && rc.rc_enable);
      bool force_upper = (upper.upper_force_active == 1);

      if (force_upper || (!rc_active && upper_ok)) {
        rt_kprintf("stop_reason :upper_ok \n");
        out_cmd_left.src = FSM_CTRL_SRC_UPPER;
        out_cmd_right.src = FSM_CTRL_SRC_UPPER;

        if (!upper_drive_ok) {
          out_cmd_left.type = CMD_STOP;
          out_cmd_left.rpm_axis1 = 0;
          out_cmd_left.rpm_axis2 = 0;
          out_cmd_right.type = CMD_STOP;
          out_cmd_right.rpm_axis1 = 0;
          out_cmd_right.rpm_axis2 = 0;
          out_st.stop_reason = FSM_STOP_TIMEOUT; /* upper cmd timeout */
          out_st.timeout_detail_code = TO_UPPER_DRIVE;
        } else {
          rc_input_t upper_mix_in;
          vehicle_config_t upper_mix_cfg;
          tune_config_t upper_mix_tune;
          calc_state_t upper_mix_state;
          motor_output_t upper_mix_out;

          upper_mix_in.throttle = (float)upper_drive.throttle_cmd;
          upper_mix_in.steering = (float)upper_drive.steering_cmd;
          upper_mix_cfg = g_rcm_vehicle;
          upper_mix_tune = g_rcm_tune;

          memset(&upper_mix_state, 0, sizeof(upper_mix_state));
          upper_mix_out = mix_rc_to_tracks(&upper_mix_in, &upper_mix_cfg, &upper_mix_tune, &upper_mix_state);

          out_cmd_left.type = CMD_SETPOINT;
          out_cmd_left.rpm_axis1 = upper_mix_out.left_input;
          out_cmd_left.rpm_axis2 = upper_mix_out.left_input;
          out_cmd_right.type = CMD_SETPOINT;
          out_cmd_right.rpm_axis1 = upper_mix_out.right_input;
          out_cmd_right.rpm_axis2 = upper_mix_out.right_input;
          (void)upper_mix_state;
        }

        out_st.control_src = FSM_CTRL_SRC_UPPER;
        if (out_st.stop_reason != FSM_STOP_TIMEOUT)
          out_st.stop_reason = FSM_STOP_REASON_NONE;
      } else if (rc_active) {
        rt_kprintf("stop_reason :rc_ok \n");
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

        rt_kprintf("stop_reason : none \n");
        out_cmd_left.src = FSM_CTRL_SRC_STOP;
        out_cmd_left.type = CMD_STOP;
        out_cmd_right.src = FSM_CTRL_SRC_STOP;
        out_cmd_right.type = CMD_STOP;
        out_st.control_src = FSM_CTRL_SRC_STOP;
        out_st.stop_reason = FSM_STOP_TIMEOUT;
        out_st.timeout_detail_code = make_timeout_detail_code(rc_timeout, upper_cfg_timeout, upper_drive_timeout,
                                                              motor_left_timeout, motor_right_timeout);
      }
    }

    // out_st.axis1_cmd = out_cmd.rpm_axis1;
    // out_st.axis2_cmd = out_cmd.rpm_axis2;

    /* Apply same default driver configuration to left/right.
     * Accept upper-supplied config only when enable bits are BOTH_ENABLE.
     */
    uint8_t driver_cfg = MOTOR_DRV_DEFAULT_ENABLE_BITS;
    if (upper_ok && ((upper.driver_config_bitmask & D0_ENABLE_MASK) == D0_EN_BOTH_ENABLE))
      driver_cfg = upper.driver_config_bitmask;

    out_cmd_left.enable_bit = driver_cfg;
    out_cmd_right.enable_bit = driver_cfg;

    out_cmd_left.axis1_accel_bit = MOTOR_DRV_DEFAULT_AXIS1_ACC;
    out_cmd_left.axis2_accel_bit = MOTOR_DRV_DEFAULT_AXIS2_ACC;
    out_cmd_right.axis1_accel_bit = MOTOR_DRV_DEFAULT_AXIS1_ACC;
    out_cmd_right.axis2_accel_bit = MOTOR_DRV_DEFAULT_AXIS2_ACC;

    /* Build feedback payloads for upper (100ms TX in can_thread). */
    upper_status_rpm_t out_rpm_st;
    memset(&out_rpm_st, 0, sizeof(out_rpm_st));
    out_rpm_st.ts = now;
    out_rpm_st.driver_left_axis1_rpm = (int16_t)clamp_i32((int32_t)motor_left_st.rpm_axis1, CMD_MIN, CMD_MAX);
    out_rpm_st.driver_left_axis2_rpm = (int16_t)clamp_i32((int32_t)motor_left_st.rpm_axis2, CMD_MIN, CMD_MAX);
    out_rpm_st.driver_right_axis1_rpm = (int16_t)clamp_i32((int32_t)motor_right_st.rpm_axis1, CMD_MIN, CMD_MAX);
    out_rpm_st.driver_right_axis2_rpm = (int16_t)clamp_i32((int32_t)motor_right_st.rpm_axis2, CMD_MIN, CMD_MAX);

    out_st.vcu_fsm_status_mask = 0;
    if (out_st.control_src == FSM_CTRL_SRC_STOP)
      out_st.vcu_fsm_status_mask |= VCU_ST_SRC_NONE;
    else if (out_st.control_src == FSM_CTRL_SRC_RC)
      out_st.vcu_fsm_status_mask |= VCU_ST_SRC_RC;
    else if (out_st.control_src == FSM_CTRL_SRC_UPPER)
      out_st.vcu_fsm_status_mask |= VCU_ST_SRC_UPPER;

    if (out_cmd_left.type == CMD_SETPOINT)
      out_st.vcu_fsm_status_mask |= VCU_ST_RUNNING;

    if (out_st.stop_reason == FSM_STOP_UPPER_FORCE)
      out_st.vcu_fsm_status_mask |= VCU_ST_STOP_UPPER;
    else if (out_st.stop_reason == FSM_STOP_RC_EMG)
      out_st.vcu_fsm_status_mask |= VCU_ST_STOP_RC_EMG;
    else if (out_st.stop_reason == FSM_STOP_MOTOR_FAULT)
      out_st.vcu_fsm_status_mask |= VCU_ST_STOP_MOTOR_FAULT;
    else if (out_st.stop_reason == FSM_STOP_TIMEOUT)
      out_st.vcu_fsm_status_mask |= VCU_ST_STOP_TIMEOUT;

    /* Command-based monitoring:
     * Integrate heading/distance from commanded left/right inputs (out_cmd),
     * not from motor feedback RPM.
     */
    update_motion_monitor(&motion_monitor, now, out_cmd_left.rpm_axis1, (int16_t)(-out_cmd_right.rpm_axis1));

    /*add to registry with cmd & status */
    rt_mutex_take(g_lock, RT_WAITING_FOREVER);
    g_latest.motor_cmd_left = out_cmd_left;
    g_latest.motor_cmd_right = out_cmd_right;
    g_latest.upper_rpm_st = out_rpm_st;
    g_latest.upper_vcu_st = out_st;
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
    }
  }
}

/* 3-2) CAN TX thread: periodic 100ms transmit */
static void can_tx_thread_entry(void* parameter) {
  (void)parameter;

  for (;;) {
    rt_thread_delay(CAN_TX_PERIOD_MS);

    motor_cmd_t cmd_left;
    motor_cmd_t cmd_right;
    upper_status_t st;
    upper_status_rpm_t st_rpm;
    vcu_motion_monitor_t mon;
    rc_intent_t rc;

    rt_mutex_take(g_lock, RT_WAITING_FOREVER);
    cmd_left = g_latest.motor_cmd_left;
    cmd_right = g_latest.motor_cmd_right;
    st = g_latest.upper_vcu_st;
    st_rpm = g_latest.upper_rpm_st;
    mon = g_latest.motion_monitor;
    rc = g_latest.rc;
    rt_mutex_release(g_lock);

    uint8_t d0[8], d1[8];

    // rt_kprintf("send CAN msg !\n");
    /* Driver 1 real operation(run signal)*/
    pack_motor_cmd(&cmd_left, d0);
    (void)can_hw_send_ext(CANID_MOTOR_CMD_DRIVER1_TX, d0, 8);
    /* Driver 2 real operation(run signal)*/
    pack_motor_cmd(&cmd_right, d0);
    (void)can_hw_send_ext(CANID_MOTOR_CMD_DRIVER2_TX, d0, 8);

    /* send vcu status to upper */
    pack_upper_status(&st, d1);
    (void)can_hw_send_ext(CANID_UPPER_STATUS_TX, d1, 8);

    /* send driver left & right feedback rpm data to upper */
    pack_upper_status_rpm(&st_rpm, d1);
    (void)can_hw_send_ext(CANID_UPPER_STATUS_RPM_TX, d1, 8);

    /* send vehicle motion status to upper */
    pack_upper_vehicle_status(&mon, d1);
    (void)can_hw_send_ext(CANID_UPPER_VEHICLE_STATUS_TX, d1, 8);

    /* send vehicle monitor/debug status to upper */
    pack_upper_vehicle_monitor(&rc, &cmd_left, &cmd_right, &mon, d1);
    (void)can_hw_send_ext(CANID_UPPER_VEHICLE_MON_TX, d1, 8);
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
