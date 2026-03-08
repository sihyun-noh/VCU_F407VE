
/*本软件SPI选用模式0的方式进行数据传输*/

#include "MySPI.h"


/*
功能：片选使能
参数：BitValue 1位片选失能 0为片选使能
*/
void MySPI_W_CS(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOD, GPIO_Pin_7, (BitAction)BitValue);
}

/*
功能：时钟信号翻转
参数：1为高电平 0为低电平
*/
void MySPI_W_SCK(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, (BitAction)BitValue);
}

/*
主机输入从机输出
功能：接收从机的一位数据
返回值：接收从机发来的一位数据
*/
uint8_t MySPI_R_MISO(void)
{
	return GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4);
}

/*
主机输出从机输入
功能：主机发送一位数据到从机
参数：1为发送高电平 0为发送低电平
*/
void MySPI_W_MOSI(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_4, (BitAction)BitValue);
}

/*
软件SPI的GPIO口初始化
功能：主机发送一位数据到从机
参数：1为发送高电平 0为发送低电平
*/
void MySPI_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	//C13为时钟口
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//D7为片选口
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//B4为MOSI
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//D4为MISO
	GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	MySPI_W_CS(1);												//片选失能
	MySPI_W_SCK(0);												//时钟失能
}


/*
片选使能
功能：选定该从机作为通信从机
*/
void MySPI_Start(void)
{
	MySPI_W_CS(0);
}

/*
片选失能
功能：片选失能，不与从机通信
*/
void MySPI_Stop(void)
{
	MySPI_W_CS(1);
}

/*
发送并接收一个字节
功能：发送一个字节的同时并接收一个字节的数据
参数：ByteSend 为要发送的一个字节
返回值：ByteReceive 返回一个从机发来的字节
*/
uint8_t MySPI_SwapByte(uint8_t ByteSend)
{
	uint8_t i, ByteReceive = 0x00;
	
	for (i = 0; i < 8; i ++)
	{
		MySPI_W_MOSI(ByteSend & (0x80 >> i));										//在采集数据前主机将数据发出去，同时从机也将数据发出来
		MySPI_W_SCK(1);																//时钟翻转为高电平，接下来从机采集主机发送的数据，同时主机采集从机发来的数据
		if (MySPI_R_MISO() == 1){ByteReceive |= (0x80 >> i);}						//主机一位一位的采集从机发来的数据，从机也将主机的数据采集到相应的地址
		MySPI_W_SCK(0);																//时钟翻转为低电平，主机和从机分别继续发送数据。
	}
	return ByteReceive;
}
