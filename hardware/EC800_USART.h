#ifndef EC800_USART_H__
#define EC800_USART_H__

#include "main.h"
#include "board.h"
#include <stdio.h>


extern uint16_t RecData;

void Init_EC800_USART(void);

void Init_EC800_USART_NVIC( void );

void EC800_USART_SendByte(uint8_t Byte);

uint32_t EC800_USART_Pow(uint32_t X, uint32_t Y);

void EC800_USART_SendNumber(uint32_t Number, uint8_t Length);

void EC800_USART_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);

void InitStartUpEC800(void);

void StartUpEC800(void);

int bsp_EC800_USART_thread(void);

static void EC800_USART_thread_entry(void* parameter);


#endif