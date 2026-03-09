#ifndef _VCU_GATEWAY_H_
#define _VCU_GATEWAY_H_

#include <stdint.h>
#include "stm32f4xx_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== IDs / Periods ===================== */
#define CANID_UPPER_STATUS_TX     0x18FF0100u /* upper status TX (temporary) */
#define CANID_UPPER_STATUS_RPM_TX 0x18FF0100u /* upper rpm status TX (temporary) */
#define CANID_MOTOR_STATUS_RX     0x18FF0021u /* motor driver status RX */

#define CANID_UPPER_CMD_RX         0x18FF0200u /* TODO: set to real upper->gateway cmd ID */
#define CANID_UPPER_CMD_RPM_RX     0x18FF0210u /* TODO: set to real rpm upper->gateway cmd ID */
#define CANID_MOTOR_CMD_DRIVER1_TX 0x18FF2100u /* TODO: set to real gateway->motor cmd ID */
#define CANID_MOTOR_CMD_DRIVER2_TX 0x18FF2200u /* TODO: set to real gateway->motor cmd ID */

#define CAN_TX_PERIOD_MS 100u /* motor cmd + upper status, 100ms */
#define FSM_PERIOD_MS    10u  /* arbitration tick */

/* Timeouts (tune if needed) */
#define UPPER_TIMEOUT_MS 500u
#define MOTOR_TIMEOUT_MS 500u
#define SBUS_TIMEOUT_MS  1000u

/*Data bit masks */
#define D0_ENABLE_MASK      (0x03u)   /*bit1:0*/
#define D0_RESET_EN         (1u << 2) /*bit2*/
#define D0_SLIDE_EN         (1u << 3) /*bit3*/
#define D0_AIXS2_SPEED_MODE (1u << 6) /*bit6: 1=speed, 0=torque*/
#define D0_AIXS1_SPEED_MODE (1u << 7) /*bit7: 1=speed, 0=torque*/

/* Enable bits value (bit0:1)*/
#define D0_EN_BOTH_DISABLE (0x00u) /*00*/
#define D0_EN_AXIS2_ONLY   (0x01u) /*01*/
#define D0_EN_AXIS1_ONLY   (0x02u) /*10*/
#define D0_EN_BOTH_ENABLE  (0x03u) /*11*/

/* ===================== Types ===================== */
typedef enum {
  SRC_NONE = 0,
  SRC_RC = 1,
  SRC_UPPER = 2,
} cmd_src_t;
typedef enum {
  UPPER_NONE = 0,
  UPPER_RPM = 1,
  UPPER_CONFIG = 2,
} cmd_upper_t;
typedef enum {
  CMD_STOP = 0,
  CMD_SETPOINT = 1,
} cmd_type_t;

/**
 * @brief  Initialize the VCU Gateway module.
 * @note   This can be registered with RT-Thread INIT_APP_EXPORT as well.
 * @return 0 on success, negative value on failure.
 */
int vcu_gateway_init(void);

/**
 * @brief  Push a received CAN frame from ISR context into the Gateway CAN RX message queue.
 * @note   Call this function inside CAN RX interrupt handler (e.g., CAN1_RX0_IRQHandler).
 * @param  ext_id  Extended CAN ID.
 * @param  data    Pointer to CAN payload bytes (up to 8 bytes).
 * @param  dlc     Data Length Code (0..8).
 */
void gateway_can_rx_push_isr(uint32_t ext_id, const uint8_t data[8], uint8_t dlc);

void gateway_can_rx_push_isr_from_rxmsg(const CanRxMsg* rx);

#ifdef __cplusplus
}
#endif

#endif /* _VCU_GATEWAY_H_ */
