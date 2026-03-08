#ifndef RS232_1_H__
#define RS232_1_H__

#include "stm32f4xx.h"                  // Device header
#include <stdio.h>


extern uint16_t RecData;

void Init_RS232_1(void);

void Init_RS232_1_NVIC( void );

void RS232_1_SendByte(uint8_t Byte);

uint32_t RS232_1_Pow(uint32_t X, uint32_t Y);

void RS232_1_SendNumber(uint32_t Number, uint8_t Length);

void RS232_1_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);


#endif

