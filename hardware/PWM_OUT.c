#include "PWM_OUT.h"


//初始化PWM波形
void Init_PWM_OUT(void)
{

	//使能时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_InternalClockConfig(TIM2);
	
	//初始化GPIO口
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;											//复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;											//推挽模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;										//不上拉不下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//配置复用
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);									//将TIM定时器与GPIO口复用上

	//配置时基单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV2;								//
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;							//向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 100-1;											//裝载值给100   ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1680-1;										//分频值给1680  发送一位的用时1ms 输出频率位1kHz  168MHZ/168KHZ = 1kHZ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;									
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	// 清除定时器更新中断标志位
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);

	
	//给时基单元上电
	TIM_Cmd(TIM2,ENABLE);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	//将输出比较模块默认初始化
	TIM_OCStructInit(&TIM_OCInitStructure);
	//配置输出比较模块
	TIM_OCInitStructure.TIM_OCMode =TIM_OCMode_PWM1;										//采用PWM1模式
	TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High ;								
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;							//输出使能
	TIM_OCInitStructure.TIM_Pulse = 50;														//设定CCR的值 设置占空比
	TIM_OC1Init(TIM2,&TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);										//设置预加载寄存器 也就是影子寄存器

}

//初始化PWM波形
void Init_PWM_OUT1(void)
{

	//APB1给到TIM定时器的频率为84MHZ
	//使能时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_InternalClockConfig(TIM3);
	
	//初始化GPIO口
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;											//复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;											//推挽模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;										//不上拉不下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	//配置复用
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_TIM3);									//将TIM定时器与GPIO口复用上

	//配置时基单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV2;								//
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;							//向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 100-1;											//裝载值给100   ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 840-1;										//分频值给1680  发送一位的用时1ms 输出频率位1kHz  168MHZ/168KHZ = 1kHZ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;									
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	//给时基单元上电
	TIM_Cmd(TIM3,ENABLE);
	

	TIM_OCInitTypeDef TIM_OCInitStructure;
	//将输出比较模块默认初始化
	TIM_OCStructInit(&TIM_OCInitStructure);
	//配置输出比较模块
	TIM_OCInitStructure.TIM_OCMode =TIM_OCMode_PWM1;										//采用PWM1模式
	TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High ;								
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;							//输出使能
	TIM_OCInitStructure.TIM_Pulse = 50;														//设定CCR的值 设置占空比
	
	TIM_OC4Init(TIM3,&TIM_OCInitStructure);
	
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);										//设置预加载寄存器 也就是影子寄存器

}


//初始化PWM波形
void Init_PWM_OUT2(void)
{

	//使能时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	TIM_InternalClockConfig(TIM1);
	
	//初始化GPIO口
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;											//复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;											//推挽模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;										//不上拉不下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	//配置复用
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1);									//将TIM定时器与GPIO口复用上

	//配置时基单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV2;								//
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;							//向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 100-1;											//裝载值给100   ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1680-1;										//分频值给1680  发送一位的用时1ms 输出频率位1kHz  168MHZ/168KHZ = 1kHZ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;									
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);
	
	//给时基单元上电
	TIM_Cmd(TIM1,ENABLE);
	

	TIM_OCInitTypeDef TIM_OCInitStructure;
	//将输出比较模块默认初始化
	TIM_OCStructInit(&TIM_OCInitStructure);
	//配置输出比较模块
	TIM_OCInitStructure.TIM_OCMode =TIM_OCMode_PWM1;										//采用PWM1模式
	TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High ;								
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;							//输出使能
	TIM_OCInitStructure.TIM_Pulse = 50;														//设定CCR的值 设置占空比
	
	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
	
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);										//设置预加载寄存器 也就是影子寄存器
	TIM_CtrlPWMOutputs(TIM1, ENABLE);														//主动输出使能
}

//初始化PWM波形
void Init_PWM_OUT3(void)
{

	//使能时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	TIM_InternalClockConfig(TIM1);
	
	//初始化GPIO口
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;											//复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;											//推挽模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;										//不上拉不下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	//配置复用
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1);									//将TIM定时器与GPIO口复用上

	//配置时基单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV2;								//
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;							//向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 100-1;											//裝载值给100   ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1680-1;										//分频值给1680  发送一位的用时1ms 输出频率位1kHz  168MHZ/168KHZ = 1kHZ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;									
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);
	
	//给时基单元上电
	TIM_Cmd(TIM1,ENABLE);
	

	TIM_OCInitTypeDef TIM_OCInitStructure;
	//将输出比较模块默认初始化
	TIM_OCStructInit(&TIM_OCInitStructure);
	//配置输出比较模块
	TIM_OCInitStructure.TIM_OCMode =TIM_OCMode_PWM1;										//采用PWM1模式
	TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High ;								
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;							//输出使能
	TIM_OCInitStructure.TIM_Pulse = 50;														//设定CCR的值 设置占空比
	
	TIM_OC4Init(TIM1,&TIM_OCInitStructure);
	
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);										//设置预加载寄存器 也就是影子寄存器
	TIM_CtrlPWMOutputs(TIM1, ENABLE);														//主动输出使能

}

//初始化PWM波形
void Init_PWM_OUT4(void)
{

	//使能时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	TIM_InternalClockConfig(TIM4);
	
	//初始化GPIO口
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;											//复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;											//推挽模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;										//不上拉不下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	//配置复用
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);									//将TIM定时器与GPIO口复用上

	//配置时基单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV2;								//
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;							//向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 100-1;											//裝载值给100   ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1680-1;										//分频值给1680  发送一位的用时1ms 输出频率位1kHz  168MHZ/168KHZ = 1kHZ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;									
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	//给时基单元上电
	TIM_Cmd(TIM4,ENABLE);
	

	TIM_OCInitTypeDef TIM_OCInitStructure;
	//将输出比较模块默认初始化
	TIM_OCStructInit(&TIM_OCInitStructure);
	//配置输出比较模块
	TIM_OCInitStructure.TIM_OCMode =TIM_OCMode_PWM1;										//采用PWM1模式
	TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High ;								
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;							//输出使能
	TIM_OCInitStructure.TIM_Pulse = 50;														//设定CCR的值 设置占空比
	
	TIM_OC4Init(TIM4,&TIM_OCInitStructure);
	
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);										//设置预加载寄存器 也就是影子寄存器
	
}


//初始化PWM波形
void Init_PWM_OUT5(void)
{

	//使能时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_InternalClockConfig(TIM2);
	
	//初始化GPIO口
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;											//复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;											//推挽模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;										//不上拉不下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	//配置复用
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_TIM2);									//将TIM定时器与GPIO口复用上

	//配置时基单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV2;								//
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;							//向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 100-1;											//裝载值给100   ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1680-1;										//分频值给1680  发送一位的用时1ms 输出频率位1kHz  168MHZ/168KHZ = 1kHZ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;									
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	//给时基单元上电
	TIM_Cmd(TIM2,ENABLE);
	

	TIM_OCInitTypeDef TIM_OCInitStructure;
	//将输出比较模块默认初始化
	TIM_OCStructInit(&TIM_OCInitStructure);
	//配置输出比较模块
	TIM_OCInitStructure.TIM_OCMode =TIM_OCMode_PWM1;										//采用PWM1模式
	TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High ;								
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;							//输出使能
	TIM_OCInitStructure.TIM_Pulse = 50;														//设定CCR的值 设置占空比
	
	TIM_OC2Init(TIM2,&TIM_OCInitStructure);
	
	TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);										//设置预加载寄存器 也就是影子寄存器

}



