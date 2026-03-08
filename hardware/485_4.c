
#include "modbus.h"
#include "485_4.h"
#include "delay.h"

int8_t flag = 0;
//如果发送接收拿不到数据一般都是初始化参数的问题
void Init_RS485_4(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);										//开启GPIOC 和UART4 的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;														//将9,10 io口 配置为复用功能
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);									//将GPIOC的10,11引脚复用为USART4
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11,GPIO_AF_UART4);
	

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;												//设置串口传输速率为9600
	USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;				//不使用流控
	USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;								//串口发送和接受模式
	USART_InitStructure.USART_Parity = USART_Parity_No;											//不使用校验
	USART_InitStructure.USART_StopBits = USART_StopBits_1;										//停止位为1
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;									//一个数据为8位
	USART_Init(UART4,&USART_InitStructure);	
	

	Init_RS485_4_NVIC();																				//初始化NVIC
	
	//第二个参数配置出错1次，会导致进中断出不来 串口使用中断接收数据要用接收的标志位寄存器作连接
	USART_ITConfig(UART4,USART_IT_RXNE,ENABLE);												//开启NVIC
	
	USART_Cmd(UART4,ENABLE);																	//给uart上电 使能uart4
}


//初始化NVIC
void Init_RS485_4_NVIC(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
}


//发送一个字节的数据
void RS485_4_SendByte(uint8_t Byte)
{
	USART_SendData(UART4, Byte);
	//等待发送数据寄存器将数据发送到移位数据寄存器，也就是等待TXE置1，
	//若数据发送到数据移位寄存器，则TXE置1，表明数据发送到移位寄存器，也表明
	//发送数据寄存器TDR为空，可以将下一个数据写入TDR了
	while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
}

//计算下个函数的被除数
uint32_t RS485_4_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}


//以字符的形式发送一个数字 如 发送数字321 则发送结果为字符形式的321
void RS485_4_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		RS485_4_SendByte(Number / RS485_4_Pow(10, Length - i - 1) % 10 + '0');
	}
}


/*****************  发送一个16位数 **********************/
void RS485_4_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
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

//使能为发送模式
void RS485_4_TxMode(void)
{
	GPIO_WriteBit(GPIOE, GPIO_Pin_12,Bit_SET);
}

//使能为接收模式
void RS485_4_RxMode(void)
{
	GPIO_WriteBit(GPIOE, GPIO_Pin_12,Bit_RESET);
}

////将printf从定向到串口
//int fputc(int ch, FILE *f)
//{
//	/* 发送一个字节数据到串口 */
//	USART_SendData(UART4, (uint8_t) ch);
//	
//	/* 等待发送完毕 */
//	while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);		

//	return (ch);
//}

////将scanf从定向到串口
//int fgetc(FILE *f)
//{
//	/* 等待串口输入数据 */
//	while (USART_GetFlagStatus(UART4, USART_FLAG_RXNE) == RESET);
//	return (int)USART_ReceiveData(UART4);
//}


//串口1的接收中断
 void UART4_IRQHandler(void)																		//重写串口接收中断 接收串口收到的数据
{	
	
	//如果接收数据寄存器非空 (RXNE == 1)
	if(USART_GetITStatus(UART4,USART_IT_RXNE) != RESET)
	{
//		RS485_4_TxMode();
//		printf("你好，来中断了！\n");
//		//RS485_4_SendByte(10);
//		Delay_Us(5);
//		RS485_4_RxMode();
//		Delay_Us(5);
		//将数据拿走 并给中断标志位置零
		RecData = USART_ReceiveData(UART4);
		flag = 1;
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
	}
}


/*串口调试助手输入的是ascii码表上的值，接收也为该值，若再将该值打印到串口，
可再次显示该值得ascii码，或者16进制，但是如果用串口将内部的值直接打印在串口上
默认会把值转化成字符形式显示，此时值会加48.
*/



