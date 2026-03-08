
/**********************************************-软件IIC-**********************************************/

#include "MyIIC.h"
#include "delay.h"
#include "bsp_i2c_ee.h"
//初始化GPIOB的8和9口为IIC的时钟口和数据口
void Init_MyIIC(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;											/*配置为开漏模式该模式只能输出0，若想输出1要在外加弱上拉电阻
																							  输出1时，mos管不导通，无法输出，但外面有上拉电阻便直接输出1，
																							于此同时，输入部分是导通的，这样的话输出1的同时，也输入了1。拉高也为
																							将主动权交由从机控制*/
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_WriteBit(GPIOA, GPIO_Pin_8,0);
	GPIO_WriteBit(GPIOC, GPIO_Pin_9,0);
}

//手动让时钟口输出1还是0实现时钟手动翻转
void MyIIC_Write_SCL(uint8_t value)
{
	GPIO_WriteBit(GPIOA, GPIO_Pin_8,(BitAction)value);
	Delay_Us(20);
}


//手动让数据口是1还是0，实现每位的数据发送
void MyIIC_Write_SDA(uint8_t value)
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_9, (BitAction)value);
	Delay_Us(20);
}

//主机接收一位数据
uint8_t MyIIC_Recv_SDA(void)
{
	uint8_t BitValue;
	BitValue = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9);
	Delay_Us(20);
	return  BitValue;
}

//iic起始时序  （无问题）
void MyIIC_Start(void)
{
	MyIIC_Write_SDA(1);
	MyIIC_Write_SCL(1);
	MyIIC_Write_SDA(0);
	MyIIC_Write_SCL(0);
}

//IIC结束时序
void MyIIC_End(void)
{
	MyIIC_Write_SDA(0);
	MyIIC_Write_SCL(1);
	MyIIC_Write_SDA(1);
}

//发送一个字节的数据   (无问题)
void MyIIC_SendByteData(uint8_t value)
{
	for(uint8_t i = 0;i<8;i++)
	{
		MyIIC_Write_SDA(value & (0x80 >> i));
		MyIIC_Write_SCL(1);
		MyIIC_Write_SCL(0);
	}
}

//接收一个字节数据
uint8_t MyIIC_RecvByteData()
{
	uint8_t i, Byte = 0x00;
	MyIIC_Write_SDA(1);																	/*将数据位拉高，主动权交由从机控制 此时从机立刻掌控SDA，
																						从机给的数据可能是1也可能是0所以当下面SCL置1时，主机直接就开始
																						接收数据了。*/
	for (i = 0; i < 8; i ++)
	{
		MyIIC_Write_SCL(1);
		if (MyIIC_Recv_SDA() == 1){Byte |= (0x80 >> i);}
		MyIIC_Write_SCL(0);
	}
	return Byte;
}

//接收应答      0为应答 1为非应答
uint8_t MyIIC_Recv_ACK(void)
{
	uint8_t ackBit;
	MyIIC_Write_SDA(1);																	//将主动权交由从机控制，然后从机会发送应答数据
	MyIIC_Write_SCL(1);																	//将时钟线拉高读取从机发送的应答数据
	ackBit =MyIIC_Recv_SDA();															//读取应答数据
	MyIIC_Write_SCL(0);																	//时钟线拉低，进行数据主动控制权变化
	return ackBit;
}

//发送应答
void MyIIC_Send_ACK(uint8_t ackBit)
{
	MyIIC_Write_SDA(ackBit);															//直接发送应答数据
	MyIIC_Write_SCL(1);																	//时钟线拉高，从机读取数据
	MyIIC_Write_SCL(0);																	//时钟线拉低，进行数据发送，或者主动控制权变化
}


/*
*********************************************************************************************************
*	函 数 名: i2c_CheckDevice
*	功能说明: 检测I2C总线设备，CPU向发送设备地址，然后读取设备应答来判断该设备是否存在
*	形    参：_Address：设备的I2C总线地址
*	返 回 值: 返回值 0 表示正确， 返回1表示未探测到
*********************************************************************************************************
*/
uint8_t i2c_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;
	MyIIC_Start();		/* 发送启动信号 */

	/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	MyIIC_SendByteData(_Address | EEPROM_I2C_WR);
	ucAck = MyIIC_Recv_ACK();	/* 检测设备的ACK应答 */

	MyIIC_End();			/* 发送停止信号 */

	return ucAck;
}


void Init_HardwareIIC(void)
{

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_I2C1);  
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_I2C1);  

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;											/*配置为开漏模式该模式只能输出0，若想输出1要在外加弱上拉电阻
																							  输出1时，mos管不导通，无法输出，但外面有上拉电阻便直接输出1，
																							于此同时，输入部分是导通的，这样的话输出1的同时，也输入了1。拉高也为
																							将主动权交由从机控制*/
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	//IIC外设结构体配置
	I2C_DeInit(I2C1);
	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;												//模式配置为IIC模式
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;												//使能外设应答
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;				//设置该单片机作为从机时可以响应7位的地址
	I2C_InitStructure.I2C_ClockSpeed = 50000;												//设置clk的频率（发送数据的快慢） 100k以下属于低频，以上属于高频，最高400k
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;									//设置占空比，因为上拉电阻为弱上拉，高频置一的时候会出现波形不好，需要配置占空比
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;												//作为从机时它的地址
	I2C_Init(I2C1, &I2C_InitStructure);

	I2C_Cmd(I2C1, ENABLE);																	//给IIC外设上电

}	











