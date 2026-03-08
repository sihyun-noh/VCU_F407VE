
#include "modbus.h"
#include "232_1_modbus.h"
#include "delay.h"
#include "modbus.h"


u8 USART_RX_BUF[USART_REC_LEN] = {0};			//接收缓冲,最大USART_REC_LEN个字节. USART_REC_LEN = 200
u8 aRxBuffer[RXBUFFERSIZE];						//接收数据暂存处    RXBUFFERSIZE = 1
//接收状态
//bit15，	接收完成标志		0为未完成 1为完成
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA = 0;							//接收状态标记	  
	
u8 count = 0;
int8_t RS232_1_flag = 0;
//如果发送接收拿不到数据一般都是初始化参数的问题
void Init_RS232_1(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);										//开启GPIOA 和USART1 的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;														//将9,10 io口 配置为复用功能
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure );
	
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);									//将GPIOA的10,11引脚复用为USART4
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10,GPIO_AF_USART1);
	

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;												//设置串口传输速率为9600
	USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;				//不使用流控
	USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;								//串口发送和接受模式
	USART_InitStructure.USART_Parity = USART_Parity_No;											//不使用校验
	USART_InitStructure.USART_StopBits = USART_StopBits_1;										//停止位为1
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;									//一个数据为8位
	USART_Init(USART1,&USART_InitStructure);	
	

	Init_RS232_1_NVIC();																				//初始化NVIC
	
	//第二个参数配置出错1次，会导致进中断出不来 串口使用中断接收数据要用接收的标志位寄存器作连接
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);												//开启NVIC
	
	USART_Cmd(USART1,ENABLE);																	//给uart上电 使能USART1
}


//初始化NVIC
void Init_RS232_1_NVIC(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
}


//发送一个字节的数据
void RS232_1_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);
	//等待发送数据寄存器将数据发送到移位数据寄存器，也就是等待TXE置1，
	//若数据发送到数据移位寄存器，则TXE置1，表明数据发送到移位寄存器，也表明
	//发送数据寄存器TDR为空，可以将下一个数据写入TDR了
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

//计算下个函数的被除数
uint32_t RS232_1_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}


//以字符的形式发送一个数字 如 发送数字321 则发送结果为字符形式的321
void RS232_1_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		RS232_1_SendByte(Number / RS232_1_Pow(10, Length - i - 1) % 10 + '0');
	}
}


/*****************  发送一个16位数 **********************/
void RS232_1_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
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


////将printf从定向到串口
//int fputc(int ch, FILE *f)
//{
//	/* 发送一个字节数据到串口 */
//	USART_SendData(USART1, (uint8_t) ch);
//	
//	/* 等待发送完毕 */
//	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		

//	return (ch);
//}

////将scanf从定向到串口
//int fgetc(FILE *f)
//{
//	/* 等待串口输入数据 */
//	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
//	return (int)USART_ReceiveData(USART1);
//}


////串口1的接收中断
// void USART1_IRQHandler(void)																		//重写串口接收中断 接收串口收到的数据
//{	
//	
//	//如果接收数据寄存器非空 (RXNE == 1)
//	if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET)
//	{
//		//printf("你好，来中断了！\n");

//		//将数据拿走 并给中断标志位置零
//		RecData = USART_ReceiveData(USART1);
//		RS232_1_flag = 1;
//		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
//	}
//}

/***********************************************************modbus接收中断***************************************************************/
//串口1的接收中断
 void USART1_IRQHandler(void)															//重写串口接收中断 接收串口收到的数据
{	
	//printf("进串口中断了\n");
	//如果接收数据寄存器非空 (RXNE == 1)
	if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET)
	{	
		
//		aRxBuffer[0] = USART_ReceiveData(USART1);
//		USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0];
//		USART_RX_STA++;
		if((USART_RX_STA&0x8000) == 0)//接收未完成
		{
			modbus_time = 0;
			
			aRxBuffer[0] = USART_ReceiveData(USART1);									//将接收的数据存入接收暂存区
			USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0];								//按字节从前往后将数据存到USART_RX_BUF数组中
			USART_RX_STA++;																//接收的字节数，同时最高位也是接收完成标志位
			
			if(USART_RX_STA>(USART_REC_LEN-1))											//接收数据错误,重新开始接收
			{
				USART_RX_STA=0;
			}
		
		}
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

/*串口调试助手输入的是ascii码表上的值，接收也为该值，若再将该值打印到串口，
可再次显示该值得ascii码，或者16进制，但是如果用串口将内部的值直接打印在串口上
默认会把值转化成字符形式显示，此时值会加48.
*/



