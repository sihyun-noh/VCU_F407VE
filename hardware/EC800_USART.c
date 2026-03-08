#include "EC800_USART.h"
#include "delay.h"

int8_t EC800_USART_flag = 0;
uint8_t EC800_USART_RecData = 0;
//如果发送接收拿不到数据一般都是初始化参数的问题
void Init_EC800_USART(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);										//开启GPIOA 和UART5 的时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);										//开启GPIOA 和UART5 的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;														//将9,10 io口 配置为复用功能
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);									//将GPIOA的10,11引脚复用为USART4
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,GPIO_AF_UART5);
	

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;												//设置串口传输速率为9600
	USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;				//不使用流控
	USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;								//串口发送和接受模式
	USART_InitStructure.USART_Parity = USART_Parity_No;											//不使用校验
	USART_InitStructure.USART_StopBits = USART_StopBits_1;										//停止位为1
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;									//一个数据为8位
	USART_Init(UART5,&USART_InitStructure);	
	

	Init_EC800_USART_NVIC();																	//初始化NVIC
	
	//第二个参数配置出错1次，会导致进中断出不来 串口使用中断接收数据要用接收的标志位寄存器作连接
	USART_ITConfig(UART5,USART_IT_RXNE,ENABLE);													//开启NVIC
	
	USART_Cmd(UART5,ENABLE);																	//给uart上电 使能UART5
}


//初始化NVIC
void Init_EC800_USART_NVIC(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
}


//发送一个字节的数据
void EC800_USART_SendByte(uint8_t Byte)
{
	USART_SendData(UART5, Byte);
	//等待发送数据寄存器将数据发送到移位数据寄存器，也就是等待TXE置1，
	//若数据发送到数据移位寄存器，则TXE置1，表明数据发送到移位寄存器，也表明
	//发送数据寄存器TDR为空，可以将下一个数据写入TDR了
	while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
}

//计算下个函数的被除数
uint32_t EC800_USART_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}


//以字符的形式发送一个数字 如 发送数字321 则发送结果为字符形式的321
void EC800_USART_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		EC800_USART_SendByte(Number / EC800_USART_Pow(10, Length - i - 1) % 10 + '0');
	}
}


/*****************  发送一个16位数 **********************/
void EC800_USART_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* 取出高八位 */
	temp_h = (ch&0XFF00)>>8;
	/* 取出低八位 */
	temp_l = ch&0XFF;
	
	/* 发送高八位 */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	/* 发送低八位 */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}


#if 0
//将printf从定向到串口
int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口 */
	USART_SendData(UART5, (uint8_t) ch);
	
	/* 等待发送完毕 */
	while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);		

	return (ch);
}

//将scanf从定向到串口
int fgetc(FILE *f)
{
	/* 等待串口输入数据 */
	while (USART_GetFlagStatus(UART5, USART_FLAG_RXNE) == RESET);
	return (int)USART_ReceiveData(UART5);
}
#endif



static void EC800_USART_thread_entry(void* parameter)
{	
	
	StartUpEC800();
	
}

static rt_thread_t EC800_USART_thread = RT_NULL;

int bsp_EC800_USART_thread(void)
{
	EC800_USART_thread =                          /* 线程控制块指针 */
    rt_thread_create( "EC800_USART",              /* 线程名字 */
                      EC800_USART_thread_entry,   /* 线程入口函数 */
                      RT_NULL,             /* 线程入口函数参数 */
                      512,                 /* 线程栈大小 */
                      3,                   /* 线程的优先级 */
                      20);                 /* 线程时间片 */
                   
    /* 启动线程，开启调度 */
   if (EC800_USART_thread != RT_NULL)
        rt_thread_startup(EC800_USART_thread);
    else
        return -1;
}


/*
	初始化EC800的开机端口
*/
void InitStartUpEC800(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);										//开启GPIOA 和UART5 的时钟

	GPIO_InitTypeDef GPIO_InitStructure;														//将9,10 io口 配置为复用功能
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure );
	
}


//开启EC800
void StartUpEC800(void)
{
	GPIO_WriteBit(GPIOA,GPIO_Pin_7,1);
}


//串口5的接收中断
 void UART5_IRQHandler(void)																		//重写串口接收中断 接收串口收到的数据
{	
	
	//如果接收数据寄存器非空 (RXNE == 1)
	if(USART_GetITStatus(UART5,USART_IT_RXNE) != RESET)
	{
		//将数据拿走 并给中断标志位置零
		EC800_USART_RecData = USART_ReceiveData(UART5);
		RS232_2_SendByte( EC800_USART_RecData);
		EC800_USART_flag = 1;
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
	}
}


/*串口调试助手输入的是ascii码表上的值，接收也为该值，若再将该值打印到串口，
可再次显示该值得ascii码，或者16进制，但是如果用串口将内部的值直接打印在串口上
默认会把值转化成字符形式显示，此时值会加48.
*/



