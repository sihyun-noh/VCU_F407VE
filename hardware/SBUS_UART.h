#ifndef __SBUS_H
#define __SBUS_H

#include <stdint.h>
#include <string.h>
#include "stm32f4xx.h"
#include "rtthread.h"
#include <stdio.h>
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dma.h"
#include "misc.h"
#include "rtdef.h"
#include <math.h>
/**************************SBUS协议相关***********************/
/*
首部（1字节）+ 数据（22字节）+ 标志位（1字节）+ 结束符（1字节）
首部：起始字节 =0000 1111b （0x0f）
数据：22 字节的数据，分别代表16个通道的数据，也即是每个通道的值用了 11 位来表示，22x8/16 = 11
这样，每个通道的取值范围为 0~2047，低位在前、高位在后
标志位：1字节，高四位从高到低依次表示：
bit7：CH17数字通道
bit6：CH16数字通道
bit5：帧丢失(Frame lost)
bit4：安全保护(Failsafe)：失控保护激活位(0x10）判断飞机是否失控
bit3~bit0：低四位不用
结束符：0x00
*/
#define  BSP_USART6_SBUS_BAUDRATE           	(100000)						// SBUS波特率
#define  BSP_USING_SBUS											 											//用于解析SBUS模式
//#define  BSP_USART6_NORMAL_MODE             								    //正常使用串口6

/***********************SBUS***************************************/

#define SBUS_MAX_VALUE                	        (1800)    //最大值
#define SBUS_MEDIAN_VALUE               				(1000)    //捕获中间值
#define SBUS_MIN_VALUE               	  				(200)    	//最小值
#define SBUS_SWITCH_VALUE                				(600)      	//作为开关使用时值量程
#define SBUS_DIFFER_VALUE                				(100)      	//误差值
#define SBUS_RANGE_VALUE                 				(800.0)     //量程
/******************************************************************/

#define SBUS_FRAME_SIZE		25
#define SBUS_INPUT_CHANNELS	16

//定义subs信号的最小值 最大值 中值 死区 以及希望转换成PWM值的范围（1000-2000）

#define SBUS_RANGE_MIN 300.0f
#define SBUS_RANGE_MAX 1700.0f
#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f
#define DEAD_RANGE_MIN 960          //死区
#define DEAD_RANGE_MAX 1040
#define SBUS_RANGE_MIDDLE 1000.0f
#define SBUS_CONNECT_FLAG 0x00

//低速与高速模式，这里用一个二段开关控制速度档位
#define LOW_SPEED 0
#define HIGH_SPEED 1

// 定义四个摇杆与拨动开关的功能
#define YAW 1
#define THROTTLE 2
#define PITCH 3
#define ROLL 4
#define SPEED_MODE 6

extern int command[20];			   //遥控器数据

typedef struct
{
	uint16_t signal[25];
	uint16_t CH1;							//通道1数值
	uint16_t CH2;							//通道2数值
	uint16_t CH3;							//通道3数值
	uint16_t CH4;							//通道4数值
	uint16_t CH5;							//通道5数值
	uint16_t CH6;							//通道6数值
	uint16_t CH7;							//通道7数值
	uint16_t CH8;							//通道8数值
	uint16_t CH9;							//通道9数值
	uint16_t CH10;							//通道10数值
	uint16_t CH11;							//通道11数值
	uint16_t CH12;							//通道12数值
	uint16_t CH13;							//通道13数值
	uint16_t CH14;							//通道14数值
	uint16_t CH15;							//通道15数值
	uint16_t CH16;							//通道16数值
	uint8_t ConnectState;				//遥控器与接收器连接状态 0=未连接，1=正常连接
}SBUS_CH_Struct;

extern SBUS_CH_Struct SBUS_CH;


//SBUS信号解析相关函数
uint8_t update_sbus(uint8_t *buf);
uint16_t sbus_to_pwm(uint16_t sbus_value);
float sbus_to_Range(uint16_t sbus_value, float p_min, float p_max);
int bsp_SBUS_thread(void);
static void SBUS_thread_entry(void *parameter);



/*-----------------------------------------------UART配置模块-----------------------------------------------*/



//串口3   TTL      遥控器SBUS     遥控器必须设置为10通道

/***************************注意***********************************

当采用STM32进行通道解析时应注意要在遥控器功能设置中将通道选择设置为10CH，
否则会出现通道不对应相应的问题

*********************************************************************/

//引脚定义
/*******************************************************/
#define BSP_USART6                             USART6

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意
* 串口1和6是      RCC_APB2PeriphClockCmd
* 串口2/3/4/5是    RCC_APB1PeriphClockCmd
*/

// 串口对应的DMA请求通道
#define  BSP_USART6_RX_DMA_CHANNEL      		DMA_Channel_5          //通道
#define  BSP_USART6_DMA_STREAM    				DMA2_Stream1           //数据流
// 外设寄存器地址
#define  BSP_USART6_DR_ADDRESS        			(&BSP_USART6->DR)
// 一次发送的数据量
#define  BSP_USART6_RBUFF_SIZE           	   (100) 
#define  BSP_SBUSS_RBUFF_SIZE           	   (25) 

																					
#define BSP_USART6_CLK                         RCC_APB2Periph_USART6
#define BSP_USART6_BAUDRATE                    (115200)  //串口波特率

#define BSP_USART6_RX_GPIO_PORT                GPIOC
#define BSP_USART6_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOC
#define BSP_USART6_RX_PIN                      GPIO_Pin_7
#define BSP_USART6_RX_AF                       GPIO_AF_USART6
#define BSP_USART6_RX_SOURCE                   GPIO_PinSource7

#define BSP_USART6_TX_GPIO_PORT                GPIOC
#define BSP_USART6_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOC
#define BSP_USART6_TX_PIN                      GPIO_Pin_6
#define BSP_USART6_TX_AF                       GPIO_AF_USART6
#define BSP_USART6_TX_SOURCE                   GPIO_PinSource6

#define BSP_USART6_IRQHandler                  USART6_IRQHandler
#define BSP_USART6_IRQ                 				 USART6_IRQn
/************************************************************/



extern rt_uint8_t USART6_Rx_Buf[BSP_USART6_RBUFF_SIZE];


void BSP_USRT3_SEND_SEM(void);
void BSP_USART6_Config(void);
void BSP_USART6_DMA_Config(void);
void BSP_USART6_DMA_Rx_Data(void);

void BSP_USART6_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void BSP_USART6_SendString( USART_TypeDef * pUSARTx, char *str);

void BSP_USART6_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);

int bsp_uart3_thread(void);

#endif /* __USART1_H */
































