
#ifndef __CAN_AGMO_H__
#define __CAN_AGMO_H__

#include <rtthread.h>
#include <stdint.h>
#include <stdbool.h>

/* StdPeriph */
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ext_func_t)(const CanRxMsg *);

typedef struct
{
    uint32_t ext_id;
    uint8_t  dlc;
    uint8_t  data[8];
} can_msg_t;


/* Init CANA module:
 *  - create CAN TX mq
 *  - start CAN TX thread
 *  - (does NOT configure CAN hardware; do that in BSP)
 */

/* Init to can message mq */
int can_mq_init(void);

/* can rx signleton*/
rt_mq_t can_rx_mq_get(void);

/* Enqueue CAN message (thread context) */
rt_err_t cana_send_async(const can_msg_t *m);

/* CAN1 IRQ handler must be linked (defined in CAN_AGMO.c) */
void A_CAN1_RX0_IRQHandler(ext_func_t ext);

/* send to CAN msg */
bool can_send_ext(uint32_t ext_id, const uint8_t data[8], uint8_t dlc);


#ifdef __cplusplus
}
#endif

#endif /* __CAN_AGMO_H__ */
