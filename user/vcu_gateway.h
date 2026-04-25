#ifndef _VCU_GATEWAY_H_
#define _VCU_GATEWAY_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== SBUS Mapping ===================== */
#define SBUS_MIN    272
#define SBUS_CENTER 992
#define SBUS_MAX    1712

#define DEADBAND 10

/* ===================== CAN IDs / Periods ===================== */
#define CANID_UPPER_STATUS_RPM_TX     0x18FF0300u /* upper feedback: motor driver status */
#define CANID_UPPER_STATUS_TX         0x18FF0310u /* upper feedback: vcu gateway status */
#define CANID_UPPER_VEHICLE_STATUS_TX 0x18FF0320u /* upper feedback: vehicle motion status */
#define CANID_UPPER_VEHICLE_MON_TX    0x18FF0330u /* upper feedback: vehicle monitor/debug status */
#define CANID_UPPER_WEED_STATUS_TX    0x18FF0340u /* upper feedback: weed actuator status */

#define CANID_UPPER_CMD_DRIVE_RX 0x18FF0200u /* upper->gateway drive cmd ID (throttle/steering) */
#define CANID_UPPER_CMD_RX       0x18FF0210u /* TODO: set to real upper->gateway cmd ID */
#define CANID_UPPER_CMD_AUTO_RX  0x18FF0220u /* upper->gateway auto cmd ID (linear speed + yaw rate) */

#define CANID_MOTOR_CMD_DRIVER1_TX    0x18FF2100u /* TODO: set to real gateway->motor cmd ID FOR LEFT */
#define CANID_MOTOR_CMD_DRIVER2_TX    0x18FF2000u /* TODO: set to real gateway->motor cmd ID FOR RIGHT */
#define CANID_MOTOR_STATUS_LEFT_RX    0x18FF0021u /* motor driver status left RX */
#define CANID_MOTOR_STATUS_RIGHT_RX   0x18FF0020u /* motor driver status right RX */
#define CANID_WEED_ACTUATOR_TX        0x18EFC800u /* weed actuator control TX */
#define CANID_WEED_ACTUATOR_STATUS_RX 0x18FF00C8u /* weed actuator feedback RX (CYL->VCU) */

#define CAN_TX_PERIOD_MS           100u /* motor cmd + upper status, 100ms */
#define FSM_PERIOD_MS              10u  /* arbitration tick */
#define WEED_ACTUATOR_TX_PERIOD_MS 250u /* periodic actuator command TX */

/* RC left toggle(CH5) exact raw positions (3-position switch). */
#define WEED_CH5_RAW_DOWN 272u  /* weed down */
#define WEED_CH5_RAW_MID  992u  /* weed mid */
#define WEED_CH5_RAW_UP   1712u /* weed up (home) */

/* CH5 mapped actuator target in mm (meaning-oriented naming). */
#define WEED_POS_DOWN_MM 180u
#define WEED_POS_MID_MM  90u
#define WEED_POS_UP_MM   0u

/* actuator position scaling:
 * docs sample: 10mm->0x0064, 200mm->0x07D0 => pos = mm * 10
 */
#define WEED_ACT_POS_MAX_MM    200u
#define WEED_ACT_POS_SCALE_X10 10u

/* ===================== Timeouts ===================== */
#define UPPER_TIMEOUT_MS         500u
#define UPPER_DRIVE_TIMEOUT_MS   1000u
#define MOTOR_TIMEOUT_MS         500u
#define SBUS_TIMEOUT_MS          1000u
#define WEED_ACTUATOR_TIMEOUT_MS 500u
#define WEED_ACTUATOR_POS_TOL_MM 3u

/* ===================== Status Bit Fields ===================== */
/* Timeout detail code for 0x18FF0310 data[7] */
#define TO_NONE        (0u)
#define TO_RC          (1u)
#define TO_UPPER_CFG   (2u)
#define TO_UPPER_DRIVE (3u)
#define TO_MOTOR_LEFT  (4u)
#define TO_MOTOR_RIGHT (5u)
#define TO_MULTIPLE    (6u)

/* RC status bit mask */
#define RC_ST_ENABLE            (1u << 0) /* rc_enable */
#define RC_ST_EMERGENCY_STOP    (1u << 1) /* rc_emergency_stop */
#define RC_ST_FAILSAFE          (1u << 2) /* failsafe */
#define RC_ST_FRESH             (1u << 3) /* real-time RC freshness (not fresh => timeout state) */
#define RC_ST_CULTIVATOR_DOWN   (1u << 4) /* cultivator_down */
#define RC_ST_CULTIVATOR_ON     (1u << 5) /* cultivator_on */
#define RC_ST_REMOTE_AUTOMATION (1u << 6) /* rc_remote_automation */
#define RC_ST_DRIVE_MODE        (1u << 7) /* rc_drive_mode (CH10/C): 0=agile, 1=stable */

/* FSM status bit mask for 0x18FF0310 data[5]
 * mode bits (one-hot):
 *  - bit0 SAFE_STOP
 *  - bit1 MANUAL_RC
 *  - bit2 AUTO_ARMED
 *  - bit3 AUTO_ACTIVE
 * stop-reason bits:
 *  - bit4 UPPER_FORCE
 *  - bit5 RC_EMG
 *  - bit6 MOTOR_FAULT
 *  - bit7 TIMEOUT
 */
#define FSM_ST_MODE_SAFE_STOP   (1u << 0)
#define FSM_ST_MODE_MANUAL_RC   (1u << 1)
#define FSM_ST_MODE_AUTO_ARMED  (1u << 2)
#define FSM_ST_MODE_AUTO_ACTIVE (1u << 3)
#define FSM_ST_STOP_UPPER_FORCE (1u << 4)
#define FSM_ST_STOP_RC_EMG      (1u << 5)
#define FSM_ST_STOP_MOTOR_FAULT (1u << 6)
#define FSM_ST_STOP_TIMEOUT     (1u << 7)

/* Backward-compat aliases (deprecated naming). */
#define VCU_ST_SRC_NONE         FSM_ST_MODE_SAFE_STOP
#define VCU_ST_SRC_RC           FSM_ST_MODE_MANUAL_RC
#define VCU_ST_SRC_UPPER        FSM_ST_MODE_AUTO_ACTIVE
#define VCU_ST_STOP_UPPER       FSM_ST_STOP_UPPER_FORCE
#define VCU_ST_STOP_RC_EMG      FSM_ST_STOP_RC_EMG
#define VCU_ST_STOP_MOTOR_FAULT FSM_ST_STOP_MOTOR_FAULT
#define VCU_ST_STOP_TIMEOUT     FSM_ST_STOP_TIMEOUT

/* ===================== Motor Driver Bit Masks ===================== */
/* Data bit masks */
#define D0_ENABLE_MASK      (0x03u)   /*bit1:0*/
#define D0_RESET_EN         (1u << 2) /*bit2*/
#define D0_SLIDE_EN         (1u << 3) /*bit3*/
#define D0_AXIS2_SPEED_MODE (1u << 6) /*bit6: 1=speed, 0=torque*/
#define D0_AXIS1_SPEED_MODE (1u << 7) /*bit7: 1=speed, 0=torque*/

/* Enable bits value (bit0:1) */
#define D0_EN_BOTH_DISABLE (0x00u) /*00*/
#define D0_EN_AXIS2_ONLY   (0x01u) /*01*/
#define D0_EN_AXIS1_ONLY   (0x02u) /*10*/
#define D0_EN_BOTH_ENABLE  (0x03u) /*11*/

/* Default motor driver command configuration */
#define MOTOR_DRV_DEFAULT_ENABLE_BITS (D0_EN_BOTH_ENABLE | D0_AXIS1_SPEED_MODE | D0_AXIS2_SPEED_MODE)
#define MOTOR_DRV_DEFAULT_AXIS1_ACC   (0x64u)
#define MOTOR_DRV_DEFAULT_AXIS2_ACC   (0x64u)

/* ===================== Auto Mix Select ===================== */
/* Select AUTO_ACTIVE wheel-command generator at compile time.
 * - UPPER_AUTO_MIX_MODE_KINEMATIC: use mix_upper_auto_cmd_to_tracks(v, yaw_rate)
 * - UPPER_AUTO_MIX_MODE_RC_MIXER:  convert (v, yaw_rate) -> pseudo(throttle, steering) -> mix_rc_to_tracks
 */
#define UPPER_AUTO_MIX_MODE_KINEMATIC (0u)
#define UPPER_AUTO_MIX_MODE_RC_MIXER  (1u)
#define UPPER_AUTO_MIX_MODE           UPPER_AUTO_MIX_MODE_KINEMATIC

/* ===================== Public Types ===================== */
/* Control source selected by FSM arbitration. */
typedef enum {
  SRC_NONE = 0,
  SRC_RC = 1,
  SRC_UPPER = 2,
  SRC_UPPER_AUTO = 3, /* upper auto cmd (linear speed + yaw rate) */
} vcu_control_src_t;
/* Upper command category (reserved for command-path typing). */
typedef enum {
  UPPER_NONE = 0,
  UPPER_RPM = 1,
  UPPER_CONFIG = 2,
} vcu_upper_cmd_t;
/* Motor command output mode. */
typedef enum {
  CMD_STOP = 0,
  CMD_SETPOINT = 1,
} vcu_cmd_type_t;

typedef enum {
  STOP_NONE = 0,
  STOP_UPPER_FORCE = 1,
  STOP_RC_EMG = 2,
  STOP_MOTOR_FAULT = 3,
  STOP_TIMEOUT = 4,
} vcu_stop_reason_t;

typedef enum {
  FSM_MODE_SAFE_STOP = 0,
  FSM_MODE_MANUAL_RC = 1,
  FSM_MODE_AUTO_ARMED = 2,
  FSM_MODE_AUTO_ACTIVE = 3,
} vcu_fsm_mode_t;

typedef struct {
  uint32_t ts_tick; /* update tick */
  bool valid;
  bool imu_yaw_rate_valid;
  int16_t left_driver_input;
  int16_t right_driver_input;
  float yaw_deg_0_360;
  float yaw_rate_deg_s;
  float imu_yaw_rate_deg_s;
  float left_speed_m_s;
  float right_speed_m_s;
  float center_speed_m_s;
  float left_distance_m;
  float right_distance_m;
  float center_distance_m;
} vcu_motion_monitor_t;

/* ===================== Compatibility Aliases ===================== */
/* Compatibility typedefs: keep existing code style/names. */
typedef vcu_control_src_t cmd_src_t;
typedef vcu_upper_cmd_t cmd_upper_t;
typedef vcu_cmd_type_t cmd_type_t;
typedef vcu_control_src_t fsm_control_src_t;
typedef vcu_stop_reason_t fsm_stop_reason_t;

/* Compatibility macros: old FSM enum labels mapped to unified values. */
#define FSM_CTRL_SRC_STOP       SRC_NONE
#define FSM_CTRL_SRC_RC         SRC_RC
#define FSM_CTRL_SRC_UPPER      SRC_UPPER
#define FSM_CTRL_SRC_UPPER_AUTO SRC_UPPER_AUTO

#define FSM_STOP_REASON_NONE STOP_NONE
#define FSM_STOP_UPPER_FORCE STOP_UPPER_FORCE
#define FSM_STOP_RC_EMG      STOP_RC_EMG
#define FSM_STOP_MOTOR_FAULT STOP_MOTOR_FAULT
#define FSM_STOP_TIMEOUT     STOP_TIMEOUT

/* ===================== Public APIs ===================== */
/**
 * @brief  Initialize the VCU Gateway module.
 * @note   This can be registered with RT-Thread INIT_APP_EXPORT as well.
 * @return 0 on success, negative value on failure.
 */
int vcu_gateway_init(void);

/**
 * @brief  Differential drive mixer for test and runtime use.
 * @param  throttle  Forward/backward command.
 * @param  steering  Left/right steering command.
 * @param  left      Output command for left side.
 * @param  right     Output command for right side.
 */
void vcu_diff_drive_mix(int16_t throttle, int16_t steering, int16_t* left, int16_t* right);

/**
 * @brief  Push a received CAN frame from ISR context into the Gateway CAN RX message queue.
 * @note   Call this function inside CAN RX interrupt handler (e.g., CAN1_RX0_IRQHandler).
 * @param  ext_id  Extended CAN ID.
 * @param  data    Pointer to CAN payload bytes (up to 8 bytes).
 * @param  dlc     Data Length Code (0..8).
 */
void gateway_can_rx_push_isr(uint32_t ext_id, const uint8_t data[8], uint8_t dlc);

void gateway_can_rx_push_isr_from_rxmsg(const CanRxMsg* rx);

/**
 * @brief  Get latest differential-drive motion monitor snapshot.
 * @param  out  Destination pointer.
 * @return 0 on success, negative on error.
 */
int vcu_gateway_get_motion_monitor(vcu_motion_monitor_t* out);

#ifdef __cplusplus
}
#endif

#endif /* _VCU_GATEWAY_H_ */
