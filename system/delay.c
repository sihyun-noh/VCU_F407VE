
#include "Delay.h"
u8 per_us;				//每1us定时器节拍
u32 per_ms;				//每1ms节拍，注意168MHz下值为168000，需要32位，移植自STM32F0，此处谨慎

void Delay_Init()
{
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);														//自动获取各个总线以及系统时钟的频率函数，由RCC_Clocks将数据引出
		//这里有个问题，本想自动化获取时钟频率的，不料在该函数中有如下声明
	/*
	HSE_VALUE is a constant defined in stm32f4xx.h file (default value
  *                25 MHz), user has to ensure that HSE_VALUE is same as the real
  *                frequency of the crystal used. Otherwise, this function may
  *                have wrong result.
	HSE_VALUE是stm32f4xx.h文件中定义的常量(默认值为25 MHz)，
	用户必须确保HSE_VALUE与所用晶体的实际频率相同。否则，该函数可能会产生错误的结果。
	本程序所用的HSE外部晶振为8MHZ，所以对应的参数要把HSE_VALUE的值改为8000000
	
	*/
	//也就是后来测试时发现延时总是为输入的3倍左右
	//后修改了库文件，可参看对应目录的readme文件获取修改记录
	if(SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000))		//1ms一次中断
		while(1);	
	
	per_ms=SysTick->LOAD;							//每1ms节拍,亦即重载值
	per_us=per_ms/1000;								//每1us定时器节拍
 
}
 
 
 
/**
  * @brief  微秒延时.
  * @param  延时的微秒数，约定范围1~390，“禁止其他值”.
  * @note   存在一定误差，主要是函数调用+部分计算.
  * @retval None
  */
void Delay_Us(u16 us)				//微秒延时
{
	u32 ticks_old=SysTick->VAL;		//前一个计数值
	u32 ticks_new;					//后一个计数值
	u16 ticks_sum=0;				//已经经过的节拍
	u16 ticks_delta=us*per_us;		//需要经过的节拍
	if(us>390) return;				//计时不允许超过390us，超过390us请使用Delay_Us_2
	while(1)
	{
		ticks_new=SysTick->VAL;	
		if(ticks_new!=ticks_old)
		{
			if(ticks_new<ticks_old)ticks_sum+=ticks_old-ticks_new;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else ticks_sum+=per_ms-ticks_new+ticks_old;	    
			ticks_old=ticks_new;
			if(ticks_sum>=ticks_delta)break;			//时间超过/等于要延迟的时间,则退出.
		}  
	}
}
 
 
 
 
/**
  * @brief  毫秒延时.
  * @param  延时的毫秒数，约定范围1~25000，“禁止其他值”.
  * @note   存在一定误差，主要是函数调用+部分计算.
  * @retval None
  */
void Delay_Ms(u16 ms)				//毫秒延时
{
	u32 ticks_old=SysTick->VAL;		//前一个计数值
	u32 ticks_new;					//后一个计数值
	u32 ticks_sum=0;				//已经经过的节拍
	u32 ticks_delta=ms*per_ms;			//需要经过的节拍
	if(ms>25000) return;				//计时不允许超过25000ms，超过25000ms请多次使用
	while(1)
	{
		ticks_new=SysTick->VAL;	
		if(ticks_new!=ticks_old)
		{
			if(ticks_new<ticks_old)ticks_sum+=ticks_old-ticks_new;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else ticks_sum+=per_ms-ticks_new+ticks_old;	    
			ticks_old=ticks_new;
			if(ticks_sum>=ticks_delta)break;			//时间超过/等于要延迟的时间,则退出.
		}
	}
}
 
 
 
 
/**
  * @brief  微秒延时，范围更大.
  * @param  延时的微秒数，约定范围1~25000000，“禁止其他值”.
  * @note   存在一定误差，主要是函数调用+部分计算.
  * @retval None
  */
void Delay_Us_2(u32 us)				//微秒延时，范围更大
{
	u32 ticks_old=SysTick->VAL;		//前一个计数值
	u32 ticks_new;					//后一个计数值
	u32 ticks_sum=0;				//已经经过的节拍
	u32 ticks_delta=us*per_us;		//需要经过的节拍
	if(us>25000000) return;				//计时不允许超过25000000us
	while(1)
	{
		ticks_new=SysTick->VAL;	
		if(ticks_new!=ticks_old)
		{
			if(ticks_new<ticks_old)ticks_sum+=ticks_old-ticks_new;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else ticks_sum+=per_ms-ticks_new+ticks_old;	    
			ticks_old=ticks_new;
			if(ticks_sum>=ticks_delta)break;			//时间超过/等于要延迟的时间,则退出.
		}  
	}
}






void Delay_us(uint32_t xus)
{
	SysTick->LOAD = 168 * xus;				//设置定时器重装值
	SysTick->VAL = 0x00;					//清空当前计数值
	SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
	SysTick->CTRL = 0x00000004;				//关闭定时器
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 