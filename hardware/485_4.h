#ifndef RS485_4_H__
#define RS485_4_H__

#include "stm32f4xx.h"                  // Device header
#include <stdio.h>


extern uint16_t RecData;

void Init_RS485_4(void);

void Init_RS485_4_NVIC( void );

void RS485_4_SendByte(uint8_t Byte);

uint32_t RS485_4_Pow(uint32_t X, uint32_t Y);

void RS485_4_SendNumber(uint32_t Number, uint8_t Length);

void RS485_4_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);

void RS485_4_TxMode(void);

void RS485_4_RxMode(void);

#endif

