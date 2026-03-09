

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

typedef struct {
  rt_tick_t ts; /* rt_tick */
  bool valid;
  bool rc_enable;         /* CH10 enable */
  bool rc_emergency_stop; /* emergency stop */
  bool cultivator_down;
  bool cultivator_on;

  int16_t axis1; /* rc Right stick up/down */
  int16_t axis2; /* rc Right stick left/right */
  int16_t axis3; /* rc Left stick up/down */
  int16_t axis4; /* rc Left stick left/right */
  bool failsafe;
} rc_intent_t;

typedef struct {
  rt_tick_t ts;
  bool valid;
  //		uint8_t mode;              	 	   /* 0 stop / 1 setpoint (example) */
  bool upper_force_stop;   /* highest priority stop */
  bool upper_force_active; /* highest priority active */
  uint8_t enable_driver_config;
  uint8_t relay_mask;
  bool cultivator_down;
  bool cultivator_on;
} upper_intent_t;

typedef struct {
  rt_tick_t ts;
  bool valid;
  int16_t driver_f_axis1_rpm;
  int16_t driver_f_axis2_rpm;
  int16_t driver_r_axis1_rpm;
  int16_t driver_r_axis2_rpm;
} upper_intent_rpm_t;

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
  uint8_t control_src;    /* 0 stop, 1 RC, 2 Upper */
  uint8_t stop_reason;    /* 0 none, 1 upper_force, 2 rc_emg, 3 motor_fault, 4 timeout */
  uint8_t flags;          /* bit0 rc_enable, bit1 rc_emg, bit2 upper_force */
  uint8_t md_f_fault_msg; /* motor driver forward */
  uint8_t md_r_fault_msg; /* motor dirver reverse */
  uint8_t relay_st;       /* Bit mask */

} upper_status_t;

typedef struct {
  rt_tick_t ts;
  int16_t driver_f_axis1_rpm;
  int16_t driver_f_axis2_rpm;
  int16_t driver_r_axis1_rpm;
  int16_t driver_r_axis2_rpm;
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
  upper_intent_rpm_t upper_cmd_rpm;
  motor_cmd_t motor_cmd;

  motor_status_t motor_f;
  motor_status_t motor_r;
  upper_status_t upper_vcu_st;
  upper_status_rpm_t upper_rpm_st;
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

#define SBUS_MIN    272
#define SBUS_CENTER 992
#define SBUS_MAX    1712

#define CMD_MIN    (-500)
#define CMD_CENTER (0)
#define CMD_MAX    (500)

#define DEADBAND 10

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

static int16_t sbus_convert_to_control(int16_t sbus_data, uint8_t data[2]) {
  int16_t value = sbus_to_cmd(sbus_data);
  pack_int16_hi_lo(value, &data[1], &data[0]);
  return value;
}

static void make_can_payload_from_sbus(int16_t sbus_data, uint8_t data[8]) {
  int16_t cmd = sbus_to_cmd(sbus_data);
  data[0] = 0xE3;
  pack_int16_hi_lo(cmd, &data[1], &data[2]);
  data[3] = 0x64;
  pack_int16_hi_lo(cmd, &data[4], &data[5]);
  data[6] = 0x64;
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

  /* Example payload:
  data[0..1]=axis1 LE, data[2..3]=axis2 LE, data[4]=mode, data[5] bit0=force_stop */
  out->enable_driver_config = (uint8_t)rx->data[0];
  out->cultivator_down = ((rx->data[1] & 0x01u) != 0u);
  out->cultivator_on = ((rx->data[2] & 0x01u) != 0u);

  out->upper_force_stop = ((rx->data[3] & 0x01u) != 0u);
  out->upper_force_active = ((rx->data[4] & 0x01u) != 0u);
  out->relay_mask = (uint8_t)rx->data[5];

  return true;
}

/* Example: decode upper rpm cmd payload (adjust to your protocol) */
static bool decode_upper_rpm_cmd(const can_frame_t* rx, upper_intent_rpm_t* out) {
  if (rx->ext_id != CANID_UPPER_CMD_RPM_RX)
    return false;

  memset(out, 0, sizeof(*out));
  out->ts = now_tick();
  out->valid = true;

  /* Example payload:
     data[0..1]=axis1 LE, data[2..3]=axis2 LE, data[4]=mode, data[5] bit0=force_stop */
  out->driver_f_axis1_rpm = (int16_t)((uint16_t)rx->data[0] << 8 | ((uint16_t)rx->data[1]));
  out->driver_f_axis2_rpm = (int16_t)((uint16_t)rx->data[2] << 8 | ((uint16_t)rx->data[3]));
  out->driver_r_axis1_rpm = (int16_t)((uint16_t)rx->data[4] << 8 | ((uint16_t)rx->data[5]));
  out->driver_r_axis2_rpm = (int16_t)((uint16_t)rx->data[6] << 8 | ((uint16_t)rx->data[7]));

  return true;
}

/* Motor status RX: 0x18FF0021 (adjust to your motor status layout) */
static bool decode_motor_status(const can_frame_t* rx, motor_status_t* out) {
  if (rx->ext_id != CANID_MOTOR_STATUS_RX)
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

/* Upper status TX 0x18FF0300 (8 bytes, temporary compact format) */
static void pack_upper_status(const upper_status_t* st, uint8_t out[8]) {
  memset(out, 0, 8);
  out[0] = st->control_src;
  out[1] = st->stop_reason;
  out[2] = st->flags;
}

/* Upper status rpm TX 0x18FF0310 (8 bytes, temporary compact format) */
static void pack_upper_status_rpm(const upper_status_rpm_t* rpm_fb, uint8_t out[8]) {
  memset(out, 0, 8);

  out[0] = (uint8_t)((rpm_fb->driver_f_axis1_rpm >> 8) & 0xFF);
  out[1] = (uint8_t)(rpm_fb->driver_f_axis1_rpm & 0xFF);
  out[2] = (uint8_t)((rpm_fb->driver_f_axis2_rpm >> 8) & 0xFF);
  out[3] = (uint8_t)(rpm_fb->driver_f_axis2_rpm & 0xFF);
  out[4] = (uint8_t)((rpm_fb->driver_r_axis1_rpm >> 8) & 0xFF);
  out[5] = (uint8_t)(rpm_fb->driver_r_axis1_rpm & 0xFF);
  out[6] = (uint8_t)((rpm_fb->driver_r_axis2_rpm >> 8) & 0xFF);
  out[7] = (uint8_t)(rpm_fb->driver_r_axis2_rpm & 0xFF);
  /*
    out[3] = (uint8_t)(st->axis1_cmd & 0xFF);
    out[4] = (uint8_t)((st->axis1_cmd >> 8) & 0xFF);
    out[5] = (uint8_t)(st->axis2_cmd & 0xFF);
    out[6] = (uint8_t)((st->axis2_cmd >> 8) & 0xFF);
  */
}

/* ===================== Threads ===================== */

/* 1) SBUS thread: update rc_intent */
static void sbus_thread_entry(void* parameter) {
  (void)parameter;
  uint8_t cmd[8] = { 0 };
  uint8_t rpm_v[2] = { 0 };
  uint8_t frame[25];
  SBUS_CH_DATA ch;

  for (;;) {
    bool failsafe = false, lost = false;

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

    /* TODO: map channels properly */
    rc.rc_enable = (ch.CH9 > 1000);         /* CH9 */
    rc.rc_emergency_stop = (ch.CH5 > 1000); /* example CH5 */

    /* axis mapping example (center=992 assumption) */
    rc.axis1 = sbus_convert_to_control(ch.CH1, rpm_v); /* CH1 */
    rc.axis2 = sbus_convert_to_control(ch.CH2, rpm_v); /* CH2 */
    rc.axis3 = sbus_convert_to_control(ch.CH3, rpm_v); /* CH3 */
    rc.axis4 = sbus_convert_to_control(ch.CH4, rpm_v); /* CH4 */

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
  upper_intent_rpm_t upper_rpm;
  motor_status_t motor_F;
  motor_status_t motor_R;

  for (;;) {
    rt_thread_delay(FSM_PERIOD_MS);
    rt_tick_t now = now_tick();

    rt_mutex_take(g_lock, RT_WAITING_FOREVER);
    rc = g_latest.rc;
    upper = g_latest.upper_cmd_config;
    upper_rpm = g_latest.upper_cmd_rpm;
    /* motor driver status */
    motor_F = g_latest.motor_f;
    motor_R = g_latest.motor_r;
    rt_mutex_release(g_lock);

    bool rc_ok = rc.valid && is_fresh_tick(now, rc.ts, SBUS_TIMEOUT_MS);
    bool upper_ok = upper.valid && is_fresh_tick(now, upper.ts, UPPER_TIMEOUT_MS);

    /* motor driver status check */
    bool motor_F_ok = motor_F.valid && is_fresh_tick(now, motor_F.ts, MOTOR_TIMEOUT_MS) && (motor_F.fault_bits == 0);
    bool motor_R_ok = motor_R.valid && is_fresh_tick(now, motor_R.ts, MOTOR_TIMEOUT_MS) && (motor_R.fault_bits == 0);

    bool upper_force_stop = upper.upper_force_stop;
    bool rc_emg = rc.rc_emergency_stop;

    /* Forward motor driver cmd */
    motor_cmd_t out_cmd_F;
    memset(&out_cmd_F, 0, sizeof(out_cmd_F));
    out_cmd_F.ts = now;

    /* Reverse motor dirver cmd */
    motor_cmd_t out_cmd_R;
    memset(&out_cmd_R, 0, sizeof(out_cmd_R));
    out_cmd_R.ts = now;

    /* upper status */
    upper_status_t out_st;
    memset(&out_st, 0, sizeof(out_st));
    out_st.ts = now;

    out_st.flags = 0;
    if (rc.rc_enable)
      out_st.flags |= (1u << 0);
    if (rc.rc_emergency_stop)
      out_st.flags |= (1u << 1);
    if (upper_force_stop)
      out_st.flags |= (1u << 2);

    out_st.md_f_fault_msg = (uint8_t)(motor_F.fault_bits & 0xFF);
    out_st.md_r_fault_msg = (uint8_t)(motor_R.fault_bits & 0xFF);

    /* STOP conditions (highest priority) */
    if (upper_force_stop) {
      // out_cmd.src = SRC_NONE; out_cmd.type = CMD_STOP;
      out_cmd_F.src = SRC_NONE;
      out_cmd_F.type = CMD_STOP;
      out_cmd_R.src = SRC_NONE;
      out_cmd_R.type = CMD_STOP;
      out_st.control_src = 0;
      out_st.stop_reason = 1;
    } else if (rc_emg) {
      // out_cmd.src = SRC_NONE; out_cmd.type = CMD_STOP;
      out_cmd_F.src = SRC_NONE;
      out_cmd_F.type = CMD_STOP;
      out_cmd_R.src = SRC_NONE;
      out_cmd_R.type = CMD_STOP;
      out_st.control_src = 0;
      out_st.stop_reason = 2;
    } else if (!motor_F_ok) {
      // out_cmd.src = SRC_NONE; out_cmd.type = CMD_STOP;
      out_cmd_F.src = SRC_NONE;
      out_cmd_F.type = CMD_STOP;
      out_cmd_R.src = SRC_NONE;
      out_cmd_R.type = CMD_STOP;
      out_st.control_src = 0;
      out_st.stop_reason = (motor_F.valid && motor_F.fault_bits != 0) ? 3 : 4;
    }
    /*
    else if (!motor_R_ok)
    {
        out_cmd_F.src = SRC_NONE; out_cmd_F.type = CMD_STOP;
        out_cmd_R.src = SRC_NONE; out_cmd_R.type = CMD_STOP;
        out_st.control_src = 0;
        out_st.stop_reason = (motor_R.valid && motor_R.fault_bits != 0) ? 5 : 6;
    }
    */
    else {
      /* Not STOP: RC enable => RC setpoint priority (Upper STOP already handled above) */
      if (rc_ok && rc.rc_enable) {
        // rt_kprintf("stop_reason :rc_ok \n");
        // out_cmd.src = SRC_RC; out_cmd.type = CMD_SETPOINT;
        out_cmd_F.src = SRC_RC;
        out_cmd_F.type = CMD_SETPOINT;
        out_cmd_R.src = SRC_RC;
        out_cmd_R.type = CMD_SETPOINT;
        // out_cmd.rpm_axis1 = rc.axis1; out_cmd.rpm_axis2 = rc.axis2;
        out_cmd_F.rpm_axis1 = rc.axis1;
        out_cmd_F.rpm_axis2 = rc.axis3;
        out_cmd_R.rpm_axis1 = rc.axis1;
        out_cmd_R.rpm_axis2 = rc.axis3;

        out_st.control_src = 1;
        out_st.stop_reason = 0;
      } else if (upper_ok) {
        // rt_kprintf("stop_reason :upper_ok \n");
        // out_cmd.src = SRC_UPPER;
        out_cmd_F.src = SRC_UPPER;
        out_cmd_R.src = SRC_UPPER;
        if (upper.upper_force_active == 0) {
          // out_cmd.type = CMD_STOP;
          // out_cmd.rpm_axis1 = 0; out_cmd.rpm_axis2 = 0;
          out_cmd_F.type = CMD_STOP;
          out_cmd_F.rpm_axis1 = 0;
          out_cmd_F.rpm_axis2 = 0;
          out_cmd_R.type = CMD_STOP;
          out_cmd_R.rpm_axis1 = 0;
          out_cmd_R.rpm_axis2 = 0;

        } else {
          // out_cmd.type = CMD_SETPOINT;
          // out_cmd.rpm_axis1 = upper.rpm_axis1; out_cmd.rpm_axis2 = upper.rpm_axis2;
          out_cmd_F.type = CMD_SETPOINT;
          out_cmd_F.rpm_axis1 = upper_rpm.driver_f_axis1_rpm;
          out_cmd_F.rpm_axis2 = upper_rpm.driver_f_axis2_rpm;
          out_cmd_R.type = CMD_SETPOINT;
          out_cmd_R.rpm_axis1 = upper_rpm.driver_r_axis1_rpm;
          out_cmd_R.rpm_axis2 = upper_rpm.driver_r_axis2_rpm;
        }
        out_st.control_src = 2;
        out_st.stop_reason = 0;
      } else {

        rt_kprintf("stop_reason : none \n");
        out_cmd_F.src = SRC_NONE;
        out_cmd_F.type = CMD_STOP;
        out_cmd_R.src = SRC_NONE;
        out_cmd_R.type = CMD_STOP;
        out_st.control_src = 0;
        out_st.stop_reason = 4;
      }
    }

    // out_st.axis1_cmd = out_cmd.rpm_axis1;
    // out_st.axis2_cmd = out_cmd.rpm_axis2;

    /*motor driver1 bit mask*/
    /* some to bit mask*/
    /* TODO*/
    out_cmd_F.enable_bit |= D0_EN_BOTH_ENABLE;
    // out_cmd.enable_bit |= D0_RESET_EN;
    // out_cmd.enable_bit |= D0_SLIDE_EN;
    out_cmd_F.enable_bit |= D0_AIXS1_SPEED_MODE;
    out_cmd_F.enable_bit |= D0_AIXS2_SPEED_MODE;

    out_cmd_F.axis1_accel_bit = 0x64;
    out_cmd_F.axis2_accel_bit = 0x64;

    /*add to registry with cmd & status */
    rt_mutex_take(g_lock, RT_WAITING_FOREVER);
    g_latest.motor_cmd = out_cmd_F;

    g_latest.upper_vcu_st = out_st;
    rt_mutex_release(g_lock);
  }
}

/* 3) CAN thread: RX parse + TX periodic(100ms) */
static void can_thread_entry(void* parameter) {
  (void)parameter;

  rt_tick_t last_tx = now_tick();

  for (;;) {
    /* 1) RX handling: poll MQ (non-blocking) */
    can_frame_t rx;
    while (rt_mq_recv(g_can_rx_mq, &rx, sizeof(rx), 0) == RT_EOK) {
      upper_intent_t up;
      if (decode_upper_cmd(&rx, &up)) {
        rt_mutex_take(g_lock, RT_WAITING_FOREVER);
        g_latest.upper_cmd_config = up;
        rt_mutex_release(g_lock);
        continue;
      }

      upper_intent_rpm_t up_rpm;
      if (decode_upper_rpm_cmd(&rx, &up_rpm)) {
        rt_mutex_take(g_lock, RT_WAITING_FOREVER);
        g_latest.upper_cmd_rpm = up_rpm;
        rt_mutex_release(g_lock);
        continue;
      }

      /* motor driver Forward status */
      motor_status_t ms_f;
      if (decode_motor_status(&rx, &ms_f)) {
        rt_mutex_take(g_lock, RT_WAITING_FOREVER);
        g_latest.motor_f = ms_f;
        rt_mutex_release(g_lock);
        continue;
      }
      /* motor driver Reverse status */
      motor_status_t ms_r;
      if (decode_motor_status(&rx, &ms_r)) {
        rt_mutex_take(g_lock, RT_WAITING_FOREVER);
        g_latest.motor_r = ms_r;
        rt_mutex_release(g_lock);
        continue;
      }
    }

    /* 2) TX periodic: 100ms */
    rt_tick_t now = now_tick();
    uint32_t dt_ms = (uint32_t)((now - last_tx) * 1000 / RT_TICK_PER_SECOND);
    if (dt_ms >= CAN_TX_PERIOD_MS) {
      last_tx = now;

      motor_cmd_t cmd;
      upper_status_t st;
      upper_status_rpm_t st_rpm;

      rt_mutex_take(g_lock, RT_WAITING_FOREVER);
      cmd = g_latest.motor_cmd;
      st = g_latest.upper_vcu_st;
      st_rpm = g_latest.upper_rpm_st;
      rt_mutex_release(g_lock);

      uint8_t d0[8], d1[8];

      /* Driver 1 real operation(run signal)*/
      pack_motor_cmd(&cmd, d0);
      (void)can_hw_send_ext(CANID_MOTOR_CMD_DRIVER1_TX, d0, 8);
      /* Driver 2 real operation(run signal)*/
      pack_motor_cmd(&cmd, d0);
      (void)can_hw_send_ext(CANID_MOTOR_CMD_DRIVER2_TX, d0, 8);

      /* send vcu status to upper */
      pack_upper_status(&st, d1);
      (void)can_hw_send_ext(CANID_UPPER_STATUS_TX, d1, 8);

      /* send driver forward & reverse feedback rpm data to upper */
      pack_upper_status_rpm(&st_rpm, d1);
      (void)can_hw_send_ext(CANID_UPPER_STATUS_TX, d1, 8);
    }

    rt_thread_delay(1);
  }
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
  g_latest.motor_cmd.type = CMD_STOP;
  g_latest.motor_cmd.src = SRC_NONE;
  rt_mutex_release(g_lock);

  /* threads */
  rt_thread_t th;

  th = rt_thread_create("sbus", sbus_thread_entry, RT_NULL, 2048, 18, 10);
  if (th)
    rt_thread_startup(th);

  th = rt_thread_create("fsm", fsm_thread_entry, RT_NULL, 2048, 16, 10);
  if (th)
    rt_thread_startup(th);

  th = rt_thread_create("can", can_thread_entry, RT_NULL, 2048, 17, 10);
  if (th)
    rt_thread_startup(th);

  return 0;
}
