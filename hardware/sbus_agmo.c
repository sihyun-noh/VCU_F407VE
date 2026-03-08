#include "SBUS_AGMO.h"
#include "CAN_AGMO.h"

#include <string.h>
#include <stdbool.h>

/* ===================== SBUS Protocol ===================== */
#define SBUS_FRAME_SIZE 25
#define SBUS_START_BYTE 0x0F
#define SBUS_END_INDEX  24

/* RT-thread object */
#define SBUS_RX_MQ_DEPTH 8


static inline bool sbus_end_ok(uint8_t endb) {
  /* Some receivers use 0x00 or 0x04 as end byte */
  return (endb == 0x00) || (endb == 0x04);
}

/* ===================== SBUS Channels (0-based) ===================== */
enum { CH1 = 0, CH2, CH3, CH4, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16 };

/* ===================== AUTO Gate (CH10 center) ===================== */
#define AUTO_CENTER     922
#define AUTO_ON_DB      50
#define AUTO_OFF_DB     70
#define AUTO_DEBOUNCE_N 3

static inline uint16_t u16_absdiff(uint16_t a, uint16_t b) {
  return (a > b) ? (a - b) : (b - a);
}

typedef enum { AUTO_BLOCKED = 0, AUTO_PASS } auto_mode_t;

typedef struct {
  auto_mode_t mode;
  uint8_t cnt_on;
  uint8_t cnt_off;
} auto_fsm_t;

static void auto_fsm_init(auto_fsm_t* f) {
  memset(f, 0, sizeof(*f));
  f->mode = AUTO_BLOCKED; /* safe default */
}

static void auto_fsm_update(auto_fsm_t* f, uint16_t ch10) {
  uint16_t d = u16_absdiff(ch10, (uint16_t)AUTO_CENTER);
  bool on = (d <= AUTO_ON_DB);
  bool off = (d >= AUTO_OFF_DB);

  if (f->mode == AUTO_BLOCKED) {
    f->cnt_off = 0;
    if (on) {
      if (++f->cnt_on >= AUTO_DEBOUNCE_N) {
        f->mode = AUTO_PASS;
        f->cnt_on = 0;
      }
    } else {
      f->cnt_on = 0;
    }
  } else {
    f->cnt_on = 0;
    if (off) {
      if (++f->cnt_off >= AUTO_DEBOUNCE_N) {
        f->mode = AUTO_BLOCKED;
        f->cnt_off = 0;
      }
    } else {
      f->cnt_off = 0;
    }
  }
}

/* ===================== Wheel Driver CAN Protocol (per your PDF) ===================== */
/* Host -> driver: ExtID 0x1801E600, 8 bytes */
#define WD_TX_EXTID 0x1801E600u

/* Data0 bits */
#define WD_EN_MASK  0x03u
#define WD_EN_NONE  0x00u
#define WD_EN_AXIS2 0x01u
#define WD_EN_AXIS1 0x02u
#define WD_EN_BOTH  0x03u

#define WD_BIT_RESET      (1u << 2)
#define WD_BIT_SKID       (1u << 3)
#define WD_BIT_AXIS2_MODE (1u << 6) /* 1 speed */
#define WD_BIT_AXIS1_MODE (1u << 7) /* 1 speed */

#define WD_DEFAULT_ACC 100
#define WD_SPEED_MAX   30000

typedef enum { WD_MODE_TORQUE = 0, WD_MODE_SPEED = 1 } wd_mode_t;

typedef struct {
  uint8_t enable;
  bool reset;
  bool skid;
  wd_mode_t axis1_mode;
  wd_mode_t axis2_mode;

  int16_t axis1; /* motor1/left */
  int16_t axis2; /* motor2/right */
  uint8_t acc1;
  uint8_t acc2;
} wd_cmd_t;

static inline uint8_t i16_hi(int16_t v) {
  return (uint8_t)((uint16_t)v >> 8);
}
static inline uint8_t i16_lo(int16_t v) {
  return (uint8_t)((uint16_t)v & 0xFF);
}

static uint8_t wd_make_data0(const wd_cmd_t* c) {
  uint8_t d0 = (c->enable & WD_EN_MASK);
  if (c->reset)
    d0 |= WD_BIT_RESET;
  if (c->skid)
    d0 |= WD_BIT_SKID;
  if (c->axis2_mode == WD_MODE_SPEED)
    d0 |= WD_BIT_AXIS2_MODE;
  if (c->axis1_mode == WD_MODE_SPEED)
    d0 |= WD_BIT_AXIS1_MODE;
  return d0;
}

static void wd_build_payload(uint8_t out[8], const wd_cmd_t* c) {
  out[0] = wd_make_data0(c);

  /* axis2 */
  out[1] = i16_hi(c->axis2);
  out[2] = i16_lo(c->axis2);
  out[3] = c->acc2;

  /* axis1 */
  out[4] = i16_hi(c->axis1);
  out[5] = i16_lo(c->axis1);
  out[6] = c->acc1;

  out[7] = 0x00; /* reserve */
}

/* ===================== SBUS Decode ===================== */
typedef struct {
  uint16_t ch[16];
} sbus_decoded_t;

static bool sbus_decode_frame(const uint8_t frame[SBUS_FRAME_SIZE], SBUS_CH_DATA *out) {
  if (!frame || !out)
    return false;
  if (frame[0] != SBUS_START_BYTE)
    return false;
  if (!sbus_end_ok(frame[SBUS_END_INDEX]))
    return false;

	if(frame[23] == SBUS_CONNECT_FLAG)  // connected status 0x00 enable, 0x0c disable
    {
			out->ConnectState = 1;
			out->CH1 = ((frame[1] | frame[2] << 8) & 0x07FF);
			out->CH2 = ((frame[2] >> 3 | frame[3] << 5) & 0x07FF);
			out->CH3 = ((frame[3] >> 6 | frame[4] << 2 | frame[5] << 10) & 0x07FF);
			out->CH4 = ((frame[5] >> 1 | frame[6] << 7) & 0x07FF);
			out->CH5 = ((frame[6] >> 4 | frame[7] << 4) & 0x07FF);
			out->CH6 = ((frame[7] >> 7 | frame[8] << 1 | frame[9] << 9) & 0x07FF);
			out->CH7 = ((frame[9] >> 2 | frame[10] << 6) & 0x07FF);
			out->CH8 = ((frame[10] >> 5 | frame[11] << 3) & 0x07FF);
			out->CH9 = ((frame[12] | frame[13] << 8) & 0x07FF);
			out->CH10 = ((frame[13] >> 3 | frame[14] << 5) & 0x07FF);
			out->CH11 = ((frame[14] >> 6 | frame[15] << 2 | frame[16] << 10) & 0x07FF);
			out->CH12 = ((frame[16] >> 1 | frame[17] << 7) & 0x07FF);
			out->CH13 = ((frame[17] >> 4 | frame[18] << 4) & 0x07FF);
			out->CH14 = ((frame[18] >> 7 | frame[19] << 1 | frame[20] << 9) & 0x07FF);
			out->CH15 = ((frame[20] >> 2 | frame[21] << 6) & 0x07FF);
			out->CH16 = ((frame[21] >> 5 | frame[22] << 3) & 0x07FF);
		}
		else{
			out->ConnectState = 0;
		}
  return true;
}

/* ===================== SBUS -> Speed mapping & mixing ===================== */
/* Adjust SBUS_CENTER for your radio (often around 992) */
#define SBUS_CENTER   992
#define SBUS_DEADBAND 20

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static int16_t sbus_to_speed_i16(uint16_t raw, int16_t out_max) {
  int32_t x = (int32_t)raw - (int32_t)SBUS_CENTER;
  if (x > -SBUS_DEADBAND && x < SBUS_DEADBAND)
    x = 0;

  /* skeleton scaling factor; tune with real data */
  int32_t scaled = (x * out_max) / 700;
  scaled = clamp_i32(scaled, -out_max, out_max);
  return (int16_t)scaled;
}

/* Differential mixing:
 *  fwd from CH2, turn from CH4
 *  left  = fwd + turn
 *  right = fwd - turn
 * axis1 = left, axis2 = right
 */
static void mix_diff_drive(int16_t fwd, int16_t turn, int16_t* axis1_left, int16_t* axis2_right) {
  int32_t l = (int32_t)fwd + (int32_t)turn;
  int32_t r = (int32_t)fwd - (int32_t)turn;
  l = clamp_i32(l, -WD_SPEED_MAX, WD_SPEED_MAX);
  r = clamp_i32(r, -WD_SPEED_MAX, WD_SPEED_MAX);
  *axis1_left = (int16_t)l;
  *axis2_right = (int16_t)r;
}

/* ===================== SBUS RX MQ ===================== */
static struct rt_messagequeue sbus_rx_mq;
static uint8_t sbus_rx_mq_pool[SBUS_RX_MQ_DEPTH * SBUS_FRAME_SIZE];

int sbus_mq_init(void) {
  return (rt_mq_init(&sbus_rx_mq, "sbus_rx", sbus_rx_mq_pool, SBUS_FRAME_SIZE, sizeof(sbus_rx_mq_pool),
                     RT_IPC_FLAG_FIFO) == RT_EOK)
             ? 0
             : -1;
}



bool get_decode_ch_data(uint8_t frame[25],  SBUS_CH_DATA *sbus_ch_frame) {

    if (!sbus_decode_frame(frame, sbus_ch_frame))
      return false;
		else
			return true;
}

bool get_decode_25b_data(uint8_t *sbus_25b_frame) {

  while (1) {
    if (rt_mq_recv(&sbus_rx_mq, sbus_25b_frame, SBUS_FRAME_SIZE, RT_WAITING_FOREVER) != RT_EOK)
      return false;
		else
			return true;
  }
}
#if 0
/* ===================== SBUS Thread (your style) ===================== */
static rt_thread_t Sbus_thread = RT_NULL;

static void Sbus_thread_entry_AGMO(void* parameter) {
  uint8_t choose = *(uint8_t*)parameter;
  (void)choose; /* optional mode select */

  uint8_t frame[SBUS_FRAME_SIZE];
  //sbus_decoded_t sb;
	uint16_t sb[16];
  auto_fsm_t auto_fsm;

  auto_fsm_init(&auto_fsm);

  while (1) {
    if (rt_mq_recv(&sbus_rx_mq, frame, SBUS_FRAME_SIZE, RT_WAITING_FOREVER) != RT_EOK)
      continue;

    if (!sbus_decode_frame(frame, sb))
      continue;


    // update CH10 gate
    auto_fsm_update(&auto_fsm, sb[CH10]);

    // build wheel driver cmd
    wd_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.enable = WD_EN_BOTH;
    cmd.reset = false;
    cmd.skid = false;
    cmd.axis1_mode = WD_MODE_SPEED;
    cmd.axis2_mode = WD_MODE_SPEED;
    cmd.acc1 = WD_DEFAULT_ACC;
    cmd.acc2 = WD_DEFAULT_ACC;

    if (auto_fsm.mode == AUTO_PASS) {
      int16_t fwd = sbus_to_speed_i16(sb[CH2], WD_SPEED_MAX);
      int16_t turn = sbus_to_speed_i16(sb[CH4], WD_SPEED_MAX);

      mix_diff_drive(fwd, turn, &cmd.axis1, &cmd.axis2);
    } else {
      /* BLOCKED => STOP */
      cmd.axis1 = 0;
      cmd.axis2 = 0;
      /* Optional: cmd.enable = WD_EN_NONE; */
    }

    /* build CAN payload */
    uint8_t payload[8];
    wd_build_payload(payload, &cmd);

    /* enqueue to CANA */
    cana_msg_t m;
    m.ext_id = WD_TX_EXTID;
    m.dlc = 8;
    memcpy(m.data, payload, 8);

    (void)cana_send_async(&m);
  }
}

/* Thread create wrapper (your style, but fixed pointer-lifetime issue) */
int bsp_Sbus_thread_AGMO(uint8_t choose) {
  uint8_t choose_arg = 0;
  choose_arg = choose;

  /* init SBUS mq if not yet */
  if (sbus_mq_init() != 0)
    return -1;

  /* ensure CANA is ready (CAN thread + mq) */
  if (cana_init() != 0)
    return -2;

  Sbus_thread = rt_thread_create("Sbus", Sbus_thread_entry_AGMO, &choose_arg, 512, 3, 20);

  if (Sbus_thread != RT_NULL) {
    rt_thread_startup(Sbus_thread);
    return 0;
  } else {
    return -3;
  }
}
#endif

/* ===================== USART6 IRQ: SBUS Frame Collector ===================== */
/* Wait for 0x0F -> collect 25B -> push to mq */

static volatile uint8_t s_sbus_buf[SBUS_FRAME_SIZE];
static volatile uint8_t s_sbus_idx = 0;
static volatile bool s_collect = false;

void A_SBUS_USART6_IRQHandler(void) {
  rt_interrupt_enter();

  if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) {
    uint8_t b = (uint8_t)USART_ReceiveData(USART6);

    if (!s_collect) {
      if (b == SBUS_START_BYTE) {
        s_collect = true;
        s_sbus_idx = 0;
        s_sbus_buf[s_sbus_idx++] = b;
      }
    } else {
      s_sbus_buf[s_sbus_idx++] = b;

      if (s_sbus_idx >= SBUS_FRAME_SIZE) {

        (void)rt_mq_send(&sbus_rx_mq, (void*)s_sbus_buf, SBUS_FRAME_SIZE);

        s_collect = false;
        s_sbus_idx = 0;
      }
    }
  }

  rt_interrupt_leave();
}
