#ifndef __modbus_H
#define __modbus_H
#include <stdio.h>	
#include "stm32f4xx.h"                  // Device header
#include "main.h"

#define RXBUFFERSIZE   1 					//缓存大小
#define USART_REC_LEN  			200  		//定义最大接收字节数 200
#define EN_USART1_RX 			1			//使能（1）/禁止（0）串口1接收

extern unsigned int modbus_time;
extern u16 USART_RX_STA;

extern u8 USART_RX_BUF[USART_REC_LEN];
extern u8 aRxBuffer[RXBUFFERSIZE];



void modbus_service(void);
void modbus_03_function(void);
void modbus_06_function(void);
void modbus_16_function(void);
void modbus_send_data(u8 *buff,u8 len);

u16 COMM_CrcValueCalc(const u8 *data,u16 length);
unsigned int CRC16(unsigned char *puchMsg,  unsigned char usDataLen);

int bsp_Modbus_thread(void);
static void Modbus_thread_entry(void *parameter);

#endif

