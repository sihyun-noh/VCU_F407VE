/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-07-24     Tanek        the first version
 */
#include "board.h"
#include "main.h"
#include "spi.h"

#define DEBUG_USART                             USART2



#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
#define RT_HEAP_SIZE 2048
static uint32_t rt_heap[RT_HEAP_SIZE];	// heap default size: 4K(1024 * 4)

RT_WEAK void *rt_heap_begin_get(void)
{
    return rt_heap;
}

RT_WEAK void *rt_heap_end_get(void)
{
    return rt_heap + RT_HEAP_SIZE;
}

#endif



/**
 * This function will initial your board.
 */
void rt_hw_board_init()
{	
	/* System Tick Configuration */
	SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);
	
    /* Call components board initial (use INIT_BOARD_EXPORT()) */
	//延时初始化
	Delay_Init();
	
	//初始化LED
	Init_LED();
	
	//初始化一路232用于调试
	Init_RS232_2();

	//初始化用于和上位机通信的232和定时器
	Init_PWM();
	Init_RS232_1();

	//初始化IIC和MPU6050
	Init_MyIIC();														//初始化软件IIC通讯
	
	//初始化姿态传感器
	Init_MPU6050();
	Init_MPU6050();
	
	//初始化温湿度传感器
	InitTH();
	
	//初始化一路can用于电机控制
	Init_CAN1();
	
	//初始化SBUS串口以及中断
	BSP_USART6_Config();

	//初始化网口
	platform_init();

	//初始化物联网串口
	Init_EC800_USART();
	InitStartUpEC800();
	
	//初始化开关量输入
	Init_IO_IN();

	//初始化adc使用DMA读取电量值
	Init_AD();
	
	//DAC初始化使用DMA和tim2输出正弦波
	DAC_Mode_Init();

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
    
#if defined(RT_USING_CONSOLE) && defined(RT_USING_DEVICE)
	rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif
    
#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
    rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
#endif
}







void SysTick_Handler(void)
{
	/* enter interrupt */
	rt_interrupt_enter();

	rt_tick_increase();

	/* leave interrupt */
	rt_interrupt_leave();
}


void rt_hw_console_output(const char *str)
{	
	/* 进入临界段 */
    rt_enter_critical();

	/* 直到字符串结束 */
    while (*str!='\0')
	{
		/* 换行 */
        if (*str=='\n')
		{
			USART_SendData(DEBUG_USART, '\r'); 
			while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
		}

		USART_SendData(DEBUG_USART, *str++); 				
		while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);	
	}	

	/* 退出临界段 */
    rt_exit_critical();
}




/**
 * This function will delay for some us.
 *
 * @param us the delay time of us
 */
void rt_hw_us_delay(rt_uint32_t us)
{
    rt_uint32_t start, now, delta, reload, us_tick;
    start = SysTick->VAL;
    reload = SysTick->LOAD;
    us_tick = SystemCoreClock / 1000000UL;//?????????80M ??1us ??80???
    do {
        now = SysTick->VAL;
        delta = start > now ? start - now : reload + start - now;//??? ????
    } while(delta < us_tick * us);
}
