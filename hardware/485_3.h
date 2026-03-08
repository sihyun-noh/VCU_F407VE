#ifndef RS485_3_H__
#define RS485_3_H__

#include "stm32f4xx.h"                  // Device header
#include <stdio.h>


extern uint16_t RecData;

void Init_RS485_3(void);

void Init_RS485_3_NVIC( void );

void RS485_3_SendByte(uint8_t Byte);

uint32_t RS485_3_Pow(uint32_t X, uint32_t Y);

void RS485_3_SendNumber(uint32_t Number, uint8_t Length);

void RS485_3_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);

void RS485_3_TxMode(void);

void RS485_3_RxMode(void);

#endif

