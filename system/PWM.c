#include "PWM.h"
#include "modbus.h"
#include "led.h"
unsigned int modbus_time;

void Init_PWM(void)
{

	//使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE);
	TIM_InternalClockConfig(TIM11);	
	
	Init_Modbus_Timer_NVIC();																//使能NVIC
	//配置时基单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV2;								//
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;							//向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 100-1;											//裝载值给100
	TIM_TimeBaseInitStructure.TIM_Prescaler = 16800-1;										//分频值给16800  发送一位的用时1ms 输出频率位1kHz  168MHZ/168KHZ = 1kHZ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;									
	TIM_TimeBaseInit(TIM11,&TIM_TimeBaseInitStructure);
	
	// 清除定时器更新中断标志位
	TIM_ClearFlag(TIM11, TIM_FLAG_Update);

	TIM_ITConfig(TIM11, TIM_IT_Update,ENABLE);												//使能更新中断
	
	//给时基单元上电
	TIM_Cmd(TIM11,ENABLE);
}


//初始化NVIC
void Init_Modbus_Timer_NVIC(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM11_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
}


//定时器中断，每隔1us触发一次，
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{	
	
	if ( TIM_GetITStatus( TIM11, TIM_IT_Update) != RESET ) 
	{	

        modbus_time++;
//																							/*接收的数据要大于4个字节，也就是5个及5个以上，因为收到的数据包
//																							从第五个开始才有数据：从机ID，功能码，起始地址2个字节，然后才是第五个字节。
//																							再者，串口每次进接收中断都会在回调函数中把modbus_time清零，也就是说当串口的接收中断
//																							不再接收后也就是没有数据再发了，定时器进入四次以上中断下面条件才可能成立，四次timer中断用时520*4us
//																							并且USART_RX_STA里面有效字节数目不为0，那么定时器将USART_RX_STA最高位置1*/
		 if(modbus_time >4 && ((USART_RX_STA&0X3FFF) !=0))																	
		 {
			USART_RX_STA|=0x8000;
		 }
		TIM_ClearITPendingBit(TIM11 , TIM_IT_Update);  		 
	}
}



