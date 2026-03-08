/**
  ******************************************************************************
  * @file    bsp_i2c_ee.c
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   i2c EEPROM(AT24C02)应用函数bsp
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F407 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "bsp_i2c_ee.h"
#include "MyIIC.h"
#include "stdlib.h"
#include "math.h"
#include "stdio.h"

/*
*********************************************************************************************************
*	函 数 名: ee_CheckOk
*	功能说明: 判断串行EERPOM是否正常
*	形    参：无
*	返 回 值: 1 表示正常， 0 表示不正常
*********************************************************************************************************
*/
uint8_t ee_CheckOk(void)
{
	if (i2c_CheckDevice(EEPROM_DEV_ADDR) == 0)
	{
		return 1;
	}
	else
	{
		/* 失败后，切记发送I2C总线停止信号 */
		MyIIC_End();		
		return 0;
	}
}

//指定地址写（一个字节数据）
char EEPROM_WriteByteData(u16 RegAddr,uint8_t Data)
{
	MyIIC_Start();														//开始发送数据
	MyIIC_SendByteData(EEPROM_DEV_ADDR | EEPROM_I2C_WR);				//发送要写入得地址
	if(MyIIC_Recv_ACK() != 0)													//从机应答
	{
		rt_kprintf("该从机地址无从机应答！\n");
		return -1;
	}
	MyIIC_SendByteData((RegAddr>>8));										//发送要写入的寄存器地址
	if(MyIIC_Recv_ACK() != 0)													//从机应答
	{
		rt_kprintf("写入的该地址出错,从机无法应答！\n");
		return -1;
	}
	MyIIC_SendByteData(((u8)RegAddr));										//发送要写入的寄存器地址
	if(MyIIC_Recv_ACK() != 0)													//从机应答
	{
		rt_kprintf("写入的该地址出错,从机无法应答！\n");
		return -1;
	}
	MyIIC_SendByteData(Data);											//发送要写入的数据
	if(MyIIC_Recv_ACK() != 0)
	{
		rt_kprintf("写入的数据有问题，从机无法应答！\n");
		return -1;
	}
	
	MyIIC_End();														//通讯结束
	rt_kprintf("写入成功！\n");
	Delay_Ms(1);															//需要延时一会才可进行下次写入
}

//指定地址读（一个字节数据）
uint8_t EEPROM_ReadByteData(u16 RegAddr)
{
	uint8_t RecValue;
	MyIIC_Start();														//开始发送数据
	MyIIC_SendByteData(EEPROM_DEV_ADDR | EEPROM_I2C_WR);				//发送要写入设备的地址
	MyIIC_Recv_ACK();													//从机应答
	MyIIC_SendByteData(0);
	MyIIC_Recv_ACK();
	MyIIC_SendByteData(RegAddr);										//发送要读出的寄存器地址
	MyIIC_Recv_ACK();													//从机应答
	
	MyIIC_Start();														//从新起始
	MyIIC_SendByteData(EEPROM_DEV_ADDR | EEPROM_I2C_RD);				//发送要读出设备的地址
	MyIIC_Recv_ACK();													//从机应答
	RecValue = MyIIC_RecvByteData();									//将数据拿到
	MyIIC_Send_ACK(1);													//收一次数据 从机交回控制权
	MyIIC_End();														//通讯结束
	//Delay_Ms(1);
	return RecValue;
}

/*
*********************************************************************************************************
*	函 数 名: ee_ReadBytes
*	功能说明: 从串行EEPROM指定地址处开始读取若干数据
*	形    参：_usAddress : 起始地址
*			 _usSize : 数据长度，单位为字节
*			 _pReadBuf : 存放读到的数据的缓冲区指针
*	返 回 值: 0 表示失败，1表示成功
*********************************************************************************************************
*/
uint8_t ee_ReadBytes(uint8_t *_pReadBuf, uint16_t _usAddress, uint16_t _usSize)
{
	uint16_t i;
	
	/* 采用串行EEPROM随即读取指令序列，连续读取若干字节 */
	
	/* 第1步：发起I2C总线启动信号 */
	MyIIC_Start();
	
	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	MyIIC_SendByteData(EEPROM_DEV_ADDR | EEPROM_I2C_WR);	/* 此处是写指令 */
	 
	/* 第3步：等待ACK */
	if (MyIIC_Recv_ACK() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第4步：发送高位字节地址 */
	MyIIC_SendByteData(((uint8_t)(_usAddress>>8)));
	
	/* 第5步：等待ACK */
	if (MyIIC_Recv_ACK() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	/* 第4步：发送低位字节地址 */
	MyIIC_SendByteData(((uint8_t)(_usAddress)));
	
	/* 第5步：等待ACK */
	if (MyIIC_Recv_ACK() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	
	/* 第6步：重新启动I2C总线。前面的代码的目的向EEPROM传送地址，下面开始读取数据 */
	MyIIC_Start();
	
	/* 第7步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	MyIIC_SendByteData(EEPROM_DEV_ADDR | EEPROM_I2C_RD);	/* 此处是读指令 */
	
	/* 第8步：发送ACK */
	if (MyIIC_Recv_ACK() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}	
	
	/* 第9步：循环读取数据 */
	for (i = 0; i < _usSize; i++)
	{
		_pReadBuf[i] = MyIIC_RecvByteData();	/* 读1个字节 */
		
		/* 每读完1个字节后，需要发送Ack， 最后一个字节不需要Ack，发Nack */
		if (i != _usSize - 1)
		{
			MyIIC_Send_ACK(0);	/* 中间字节读完后，CPU产生ACK信号(驱动SDA = 0) */
		}
		else
		{
			MyIIC_Send_ACK(1);	/* 最后1个字节读完后，CPU产生NACK信号(驱动SDA = 1) */
		}
	}
	/* 发送I2C总线停止信号 */
	MyIIC_End();
	return 1;	/* 执行成功 */

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	MyIIC_End();
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: ee_WriteBytes
*	功能说明: 向串行EEPROM指定地址写入若干数据，采用页写操作提高写入效率
*	形    参：_usAddress : 起始地址
*			 _usSize : 数据长度，单位为字节
*			 _pWriteBuf : 存放写到的数据的缓冲区指针
*	返 回 值: 0 表示失败，1表示成功
*********************************************************************************************************
*/
uint8_t ee_WriteBytes(uint8_t *_pWriteBuf, uint16_t _usAddress, uint16_t _usSize)
{
	uint16_t i,m;
	uint16_t usAddr;
	
	/* 
		写串行EEPROM不像读操作可以连续读取很多字节，每次写操作只能在同一个page。
		对于24xx02，page size = 512
		简单的处理方法为：按字节写操作模式，每写1个字节，都发送地址
		为了提高连续写的效率: 本函数采用page wirte操作。
	*/

	usAddr = _usAddress;	
	for (i = 0; i < _usSize; i++)
	{
		/* 当发送第1个字节或是页面首地址时，需要重新发起启动信号和地址 */
		if ((i == 0) || ((usAddr & (EEPROM_PAGE_SIZE - 1)) == 0))
		{
			/*　第０步：发停止信号，启动内部写操作　*/
			MyIIC_End();
			
			/* 通过检查器件应答的方式，判断内部写操作是否完成, 一般小于 10ms 			
				CLK频率为200KHz时，查询次数为30次左右
			*/
			for (m = 0; m < 1000; m++)
			{				
				/* 第1步：发起I2C总线启动信号 */
				MyIIC_Start();
				
				/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
				MyIIC_SendByteData(EEPROM_DEV_ADDR | EEPROM_I2C_WR);	/* 此处是写指令 */
				
				/* 第3步：发送一个时钟，判断器件是否正确应答 */
				if (MyIIC_Recv_ACK() == 0)
				{
					break;
				}
			}
			if (m  == 1000)
			{
				goto cmd_fail;	/* EEPROM器件写超时 */
			}
		
			/* 第4步：发送高位字节地址 */
			MyIIC_SendByteData(((uint8_t)(usAddr>>8)));
			
			/* 第5步：等待ACK */
			if (MyIIC_Recv_ACK() != 0)
			{
				goto cmd_fail;	/* EEPROM器件无应答 */
			}
			
			/* 发送低位字节地址 */
			MyIIC_SendByteData((uint8_t)usAddr);
			
			/* 等待ACK */
			if (MyIIC_Recv_ACK() != 0)
			{
				goto cmd_fail;	/* EEPROM器件无应答 */
			}
		}
	
		/* 第6步：开始写入数据 */
		MyIIC_SendByteData(_pWriteBuf[i]);
	
		/* 第7步：等待接收ACK */
		if (MyIIC_Recv_ACK() != 0)
		{
			goto cmd_fail;	/* EEPROM器件无应答 */
		}

		usAddr++;	/* 地址增1 */		
	}
	
	/* 命令执行成功，发送I2C总线停止信号 */
	MyIIC_End();
	return 1;

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	MyIIC_End();
	return 0;
}


void ee_Erase(void)
{
	uint16_t i;
	uint8_t buf[EEPROM_SIZE];
	
	/* 填充缓冲区 */
	for (i = 0; i < EEPROM_SIZE; i++)
	{
		buf[i] = 0xFF;
	}
	
	/* 写EEPROM, 起始地址 = 0，数据长度为 256 */
	if (ee_WriteBytes(buf, 0, EEPROM_SIZE) == 0)
	{
		rt_kprintf("擦除eeprom出错！\r\n");
		return;
	}
	else
	{
		rt_kprintf("擦除eeprom成功！\r\n");
	}
}


/*--------------------------------------------------------------------------------------------------*/
static void ee_Delay(__IO uint32_t nCount)	 //简单的延时函数
{
	for(; nCount != 0; nCount--);
}


/*
 * eeprom AT24C02 读写测试
 * 正常返回1，异常返回0
 */
uint8_t ee_Test(void) 
{
	uint16_t i;
	uint8_t write_buf[EEPROM_SIZE];
	uint8_t read_buf[EEPROM_SIZE];
  
/*-----------------------------------------------------------------------------------*/  
  if (ee_CheckOk() == 0)
	{
		/* 没有检测到EEPROM */
		rt_kprintf("没有检测到串行EEPROM!\r\n");
				
		return 0;
	}
/*------------------------------------------------------------------------------------*/  
  /* 填充测试缓冲区 */
	for (i = 0; i < EEPROM_SIZE; i++)
	{		
		write_buf[i] = i;
	}
/*------------------------------------------------------------------------------------*/  
  if (ee_WriteBytes(write_buf, 0, EEPROM_SIZE) == 0)
	{
		rt_kprintf("写eeprom出错！\r\n");
		return 0;
	}
	else
	{		
		rt_kprintf("写eeprom成功！\r\n");
	}
  
  /*写完之后需要适当的延时再去读，不然会出错*/
  //ee_Delay(0x0FFFFF);
	Delay_Ms(10);
/*-----------------------------------------------------------------------------------*/
  if (ee_ReadBytes(read_buf, 0, EEPROM_SIZE) == 0)
	{
		rt_kprintf("读eeprom出错！\r\n");
		return 0;
	}
	else
	{		
		rt_kprintf("读eeprom成功，数据如下：\r\n");
	}
/*-----------------------------------------------------------------------------------*/  
  for (i = 0; i < EEPROM_SIZE; i++)
	{
//		if(read_buf[i] != write_buf[i])
//		{
//			rt_kprintf("0x%02X ", read_buf[i]);
//			rt_kprintf("错误:EEPROM读出与写入的数据不一致");
//			return 0;
//		}
    rt_kprintf(" %02X", read_buf[i]);
		
		if ((i & 15) == 15)
		{
			rt_kprintf("\r\n");	
		}		
	}
	rt_kprintf("eeprom读写测试成功\r\n");

  return 1;
}


/*
功能：开启线程通过sbus和can控制小车行走
参数：
*/

//u8 arr[10] = {1,3,4,6,8,0,4,3,2,1};
//u8 rec[10] = {0};
static void Ee_thread_entry(void *parameter)
{

	
	//写入机器人型号：10000
	EEPROM_WriteByteData(0x15,0x27);
	EEPROM_WriteByteData(0x16,0x10);
	//驱动器版本型号
	EEPROM_WriteByteData(0x17,0x00);
	EEPROM_WriteByteData(0x18,0x01);
	EEPROM_WriteByteData(0x19,0x00);
	EEPROM_WriteByteData(0x1A,0x02);
	EEPROM_WriteByteData(0x1B,0x00);
	EEPROM_WriteByteData(0x1C,0x03);
	EEPROM_WriteByteData(0x1D,0x00);
	EEPROM_WriteByteData(0x1E,0x04);
	while(1)
	{

		rt_thread_delay(1000);										//在main函数运行开启线程的时候需要线程让出CPU资源，保证cpu可以运行开启后面的线程。
	}
}


/* 定义线程控制块 */
static rt_thread_t Ee_thread = RT_NULL;

int bsp_Ee_thread(void)
{
	
	Ee_thread =                          /* 线程控制块指针 */
    rt_thread_create( "Ee",              /* 线程名字 */
                      Ee_thread_entry,   /* 线程入口函数 */
                      RT_NULL,             /* 线程入口函数参数 */
                      512,                 /* 线程栈大小 */
                      3,                   /* 线程的优先级 */
                      20);                 /* 线程时间片 */
                   
    /* 启动线程，开启调度 */
   if (Ee_thread != RT_NULL)
        rt_thread_startup(Ee_thread);
    else
        return -1;   

}



/*********************************************END OF FILE**********************/
