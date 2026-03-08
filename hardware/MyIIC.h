
/**********************************************-Èí¼þIIC-**********************************************/


#ifndef MYIIC_H__
#define MYIIC_H__

#include "stm32f4xx.h"                  // Device header


				

void Init_MyIIC(void);
void MyIIC_Write_SCL(uint8_t value);
void MyIIC_Write_SDA(uint8_t value);
uint8_t MyIIC_Recv_SDA(void);
void MyIIC_Start(void);
void MyIIC_End(void);
void MyIIC_SendByteData(uint8_t value);
uint8_t MyIIC_RecvByteData();
uint8_t MyIIC_Recv_ACK(void);
void MyIIC_Send_ACK(uint8_t ackBit);
uint8_t i2c_CheckDevice(uint8_t _Address);

void Init_HardwareIIC(void);
#endif

