
#include "PWM_IN.h"


//初始化输入捕获通道
void Init_PWM_IN1(void)
{

	//使能时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	TIM_InternalClockConfig(TIM5);
	
	//初始化GPIO口
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;											//复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;											//推挽模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;										//不上拉不下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//配置复用
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);									//将TIM定时器与GPIO口复用上

	//配置时基单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV2;								//
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;							//向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 65536-1;											//裝载值给100   ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 168-1;										//分频值给1680  发送一位的用时1ms 输出频率位1kHz  168MHZ/168KHZ = 1kHZ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;									
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
	
	//给时基单元上电
	TIM_Cmd(TIM5,ENABLE);
	
	//初始化输入捕获模块
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;										//选用TIM5的第二个通道
	TIM_ICInitStructure.TIM_ICFilter = 0xF;													//用来选用输入捕获的滤波器，数值越大滤波效果越好
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;								//选择采样的边沿，可选择上升沿采样，下降沿采样，双边沿采样
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;									//对输入信号进行分频
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;							//选择出发信号从那个引脚输入，该项是配置数据选择器的，选择是直连通道还是交叉通道
	TIM_PWMIConfig(TIM5,&TIM_ICInitStructure);												//该函数就等同于下面注释的代码

	/*
	要想使用测周法测量输入信号的占空比，需要再开一路通道，配置该通道的
	边沿检测为下降沿，则cnt在下降沿到来时便会将数据存入这一通道的CCR里面。
	本配置中为CCR1，本配置使用的是交叉连接的通道1，也就是使用从通道二引
	脚进来的PWM信号，将其引入通道1中，然后在通道1中配置下降沿边沿检测后，
	产生信号TI2FP1，经过分频到达通道1的捕获寄存器（本程序不分频）在输入捕
	获寄存器中，CCR1将拿到检测到下降沿后cnt的数值。
	*/
//	TIM_ICInit(TIM5,&TIM_ICInitStructure);
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;										//选用TIM5的第二个通道
//	TIM_ICInitStructure.TIM_ICFilter = 0xF;													//用来选用输入捕获的滤波器，数值越大滤波效果越好
//	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;								//选择采样的边沿，可选择上升沿采样，下降沿采样，双边沿采样
//	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;									//对输入信号进行分频
//	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;							//选择出发信号从那个引脚输入，该项是配置数据选择器的，选择是直连通道还是交叉通道
//	TIM_ICInit(TIM5,&TIM_ICInitStructure);
																							
	TIM_SelectInputTrigger(TIM5,TIM_TS_TI2FP2);												//配置输入触发信号为TI2FP2，也就是通道二直连IC2的信号
	TIM_SelectSlaveMode(TIM5,TIM_SlaveMode_Reset);											//配置从模式为复位cnt的值，当接收到触发信号TI2FP2时复位cnt的值

}


//初始化输入捕获通道
void Init_PWM_IN2(void)
{

	//使能时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	TIM_InternalClockConfig(TIM1);
	
	//初始化GPIO口
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;											//复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;											//推挽模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;										//不上拉不下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	//配置复用
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1);									//将TIM定时器与GPIO口复用上

	//配置时基单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV2;								//
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;							//向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 65536-1;											//裝载值给100   ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 168-1;										//分频值给1680  发送一位的用时1ms 输出频率位1kHz  168MHZ/168KHZ = 1kHZ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;									
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);
	
	//给时基单元上电
	TIM_Cmd(TIM1,ENABLE);
	
	//初始化输入捕获模块
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;										//选用TIM1的第1个通道
	TIM_ICInitStructure.TIM_ICFilter = 0xF;													//用来选用输入捕获的滤波器，数值越大滤波效果越好
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;								//选择采样的边沿，可选择上升沿采样，下降沿采样，双边沿采样
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;									//对输入信号进行分频
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;							//选择出发信号从那个引脚输入，该项是配置数据选择器的，选择是直连通道还是交叉通道
	TIM_ICInit(TIM1,&TIM_ICInitStructure);

																							
	TIM_SelectInputTrigger(TIM1,TIM_TS_TI1FP1);												//配置输入触发信号为TI1FP1，也就是通道二直连IC2的信号
	TIM_SelectSlaveMode(TIM1,TIM_SlaveMode_Reset);											//配置从模式为复位cnt的值，当接收到触发信号TI1FP1时复位cnt的值

}

//初始化输入捕获通道
void Init_PWM_IN3(void)
{

	//使能时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	TIM_InternalClockConfig(TIM1);
	
	//初始化GPIO口
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;											//复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;											//推挽模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;										//不上拉不下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	//配置复用
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1);									//将TIM定时器与GPIO口复用上

	//配置时基单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV2;								//
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;							//向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 65536-1;											//裝载值给100   ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 168-1;										//分频值给1680  发送一位的用时1ms 输出频率位1kHz  168MHZ/168KHZ = 1kHZ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;									
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);
	
	//给时基单元上电
	TIM_Cmd(TIM1,ENABLE);
	
	//初始化输入捕获模块
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;										//选用TIM1的第1个通道
	TIM_ICInitStructure.TIM_ICFilter = 0xF;													//用来选用输入捕获的滤波器，数值越大滤波效果越好
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;								//选择采样的边沿，可选择上升沿采样，下降沿采样，双边沿采样
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;									//对输入信号进行分频
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;							//选择出发信号从那个引脚输入，该项是配置数据选择器的，选择是直连通道还是交叉通道
	TIM_ICInit(TIM1,&TIM_ICInitStructure);

																							
	TIM_SelectInputTrigger(TIM1,TIM_TS_TI1FP1);												//配置输入触发信号为TI1FP1，也就是通道二直连IC2的信号
	TIM_SelectSlaveMode(TIM1,TIM_SlaveMode_Reset);											//配置从模式为复位cnt的值，当接收到触发信号TI1FP1时复位cnt的值

}


//初始化输入捕获通道
void Init_PWM_IN4(void)
{

	//使能时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	TIM_InternalClockConfig(TIM4);
	
	//初始化GPIO口
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;											//复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;											//推挽模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;										//不上拉不下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	//配置复用
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);									//将TIM定时器与GPIO口复用上

	//配置时基单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV2;								//
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;							//向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 65536-1;											//裝载值给100   ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 168-1;										//分频值给1680  发送一位的用时1ms 输出频率位1kHz  168MHZ/168KHZ = 1kHZ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;									
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	//给时基单元上电
	TIM_Cmd(TIM4,ENABLE);
	
	//初始化输入捕获模块
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;										//选用TIM1的第1个通道
	TIM_ICInitStructure.TIM_ICFilter = 0xF;													//用来选用输入捕获的滤波器，数值越大滤波效果越好
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;								//选择采样的边沿，可选择上升沿采样，下降沿采样，双边沿采样
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;									//对输入信号进行分频
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;							//选择出发信号从那个引脚输入，该项是配置数据选择器的，选择是直连通道还是交叉通道
	TIM_ICInit(TIM4,&TIM_ICInitStructure);

																							
	TIM_SelectInputTrigger(TIM4,TIM_TS_TI1FP1);												//配置输入触发信号为TI1FP1，也就是通道二直连IC2的信号
	TIM_SelectSlaveMode(TIM4,TIM_SlaveMode_Reset);											//配置从模式为复位cnt的值，当接收到触发信号TI1FP1时复位cnt的值

}

//初始化输入捕获通道
void Init_PWM_IN5(void)
{

	//使能时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_InternalClockConfig(TIM3);
	
	//初始化GPIO口
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;											//复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;											//推挽模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;										//不上拉不下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	//配置复用
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM3);									//将TIM定时器与GPIO口复用上

	//配置时基单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV2;								//
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;							//向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 65536-1;											//裝载值给100   ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 168-1;										//分频值给1680  发送一位的用时1ms 输出频率位1kHz  168MHZ/168KHZ = 1kHZ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;									
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	//给时基单元上电
	TIM_Cmd(TIM3,ENABLE);
	
	//初始化输入捕获模块
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;										//选用TIM1的第1个通道
	TIM_ICInitStructure.TIM_ICFilter = 0xF;													//用来选用输入捕获的滤波器，数值越大滤波效果越好
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;								//选择采样的边沿，可选择上升沿采样，下降沿采样，双边沿采样
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;									//对输入信号进行分频
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;							//选择出发信号从那个引脚输入，该项是配置数据选择器的，选择是直连通道还是交叉通道
	TIM_ICInit(TIM3,&TIM_ICInitStructure);

																							
	TIM_SelectInputTrigger(TIM3,TIM_TS_TI1FP1);												//配置输入触发信号为TI1FP1，也就是通道二直连IC2的信号
	TIM_SelectSlaveMode(TIM3,TIM_SlaveMode_Reset);											//配置从模式为复位cnt的值，当接收到触发信号TI1FP1时复位cnt的值

}

u16 Get_Input1Freq()
{
	return 1000000/TIM_GetCapture2(TIM5);
}

//获取第二路输入的PWM的频率
u16 Get_Input2Freq()
{
	return 1000000/TIM_GetCapture1(TIM1);
}




u32 Get_Input1Duty()
{
	return TIM_GetCapture1(TIM5)*100 / TIM_GetCapture2(TIM5);
}

