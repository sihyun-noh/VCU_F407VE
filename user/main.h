#ifndef MAIN_H__
#define MAIN_H__


#include "stm32f4xx.h"                  // Device header
#include "delay.h"
#include "AD.h"
#include "LED.h"
#include "PWM.h"
#include "CAN.h"
#include "stm32f4xx_can.h"
#include "dac.h"
#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "MyIIC.h"
#include "bsp_i2c_ee.h"
#include "modbus.h"
#include "485_4.h"
#include "485_3.h"
#include "232_2.h"
#include "232_1_modbus.h"
#include "can2.h"
#include "IO_10_Out.h"
#include "TH.h"
#include "PWM_OUT.h"
#include "PWM_IN.h"
#include "MySPI.h"
#include "bsp_sd.h"	
#include "SBUS_UART.h"
//#include "spi.h"
#include "socket.h"	// Just include one header for WIZCHIP
#include "string.h"
#include "EC800_USART.h"
#include "IO_10_IN.h"

#include "CAN_AGMO.h"
#include "SBUS_AGMO.h"

#endif


