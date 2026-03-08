#ifndef TH_H__
#define TH_H__

#include "stm32f4xx.h"                  			// Device header
#include "MyIIC.h"
#include "232_2.h"
#include "board.h"
#define         	TH_ADDERSS			(uint8_t)0x80         	//温湿度传感器的地址
#define				TH_I2C_WR			(uint8_t)0x00			//写功能
#define				TH_I2C_DR			(uint8_t)0x01			//读功能
#define				RESET_DRDY 			(uint8_t)0x0E			//软复位和中断配置寄存器地址
#define				MEASUREMENT			(uint8_t)0x0F			//测量配置寄存器地址

#define				DEVICE_ID_LOW			(uint8_t)0xFE			//设备ID低
#define				DEVICE_ID_HIGH			(uint8_t)0xFF			//设备ID高

#define				TH_TEMPL			(uint8_t)0x00			//温度寄存器低位地址
#define				TH_TEMPH			(uint8_t)0x01			//温度寄存器高位地址
#define				TH_HUMIL			(uint8_t)0x02			//湿度寄存器低位地址
#define				TH_HUMIH			(uint8_t)0x03			//湿度寄存器高位地址

static rt_thread_t test_thread = RT_NULL;


//初始化温湿度传感器
void InitTH(void);
//读取设备ID
u16 TH_ID(void);
//检查设备
uint8_t TH_CheckDevice(uint8_t _Address);
//向从机写数据
char TH_WriteByteData(uint8_t RegAddr,uint8_t Data);
//指定地址读（一个字节数据）
uint8_t TH_ReadByteData(uint8_t RegAddr);
//将温湿度数据读出
char GetTHData(float *TH_Data);
//将温湿度数据打印出来
void TH_PrintValue(float *TH_Data);

static void test_thread_entry(void* parameter);

int bsp_TH_thread(void);
#endif

