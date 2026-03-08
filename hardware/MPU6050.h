#ifndef MPU6050_H__
#define MPU6050_H__

#include "stm32f4xx.h"                  // Device header
#include "board.h"
#include "MyIIC.h"
#include "MPU6050_Reg.h"
#include "stdio.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "delay.h"
#define MPU6050_ADDRESS		0xD0							//设备的地址






char MPU6050_WriteByteData(uint8_t RegAddr,uint8_t Data);

uint8_t MPU6050_ReadByteData(uint8_t RegAddr);

void Init_MPU6050(void);

void MPU6050_ReadData(float *AccGyroValue);

uint8_t MPU6050_CheckDevice(uint8_t _Address);

static void Kprintf_thread_entry(float *parameter);

int bsp_MPU6050_thread(void);
#endif