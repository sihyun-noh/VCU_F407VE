
#ifndef __SBUS_AGMO_H__
#define __SBUS_AGMO_H__

#include <rtthread.h>
#include <stdint.h>
#include <stdbool.h>

/* StdPeriph */
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SBUS_CONNECT_FLAG 0x00

typedef struct {
  uint16_t signal[25];
  uint16_t CH1;
  uint16_t CH2;
  uint16_t CH3;
  uint16_t CH4;
  uint16_t CH5;
  uint16_t CH6;
  uint16_t CH7;
  uint16_t CH8;
  uint16_t CH9;
  uint16_t CH10;
  uint16_t CH11;
  uint16_t CH12;
  uint16_t CH13;
  uint16_t CH14;
  uint16_t CH15;
  uint16_t CH16;
  uint8_t ConnectState;
} SBUS_CH_DATA;

/* Create SBUS thread and start decoding.
 * choose: user parameter (mode select, optional)
 */
int bsp_Sbus_thread_AGMO(uint8_t choose);

/* Optional: init SBUS RX message queue (called inside bsp_Sbus_thread typically) */
int sbus_mq_init(void);

bool get_decode_25b_data(uint8_t *sbus_25b_frame);

bool get_decode_ch_data(uint8_t *frame, SBUS_CH_DATA *sbus_ch_frame);

/* USART6 IRQ handler must be linked (defined in SBUS_AGMO.c) */
void A_SBUS_USART6_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __SBUS_H__ */
