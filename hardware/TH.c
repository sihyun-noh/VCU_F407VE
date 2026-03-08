#include "TH.h"

//温度和湿度值
float THData[2] = {0};


uint8_t TH_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;
	MyIIC_Start();		/* 发送启动信号 */

	/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	MyIIC_SendByteData(_Address | 0x00);
	ucAck = MyIIC_Recv_ACK();	/* 检测设备的ACK应答 */

	MyIIC_End();			/* 发送停止信号 */

	return ucAck;
}


//指定地址写（一个字节数据）
char TH_WriteByteData(uint8_t RegAddr,uint8_t Data)
{

	MyIIC_Start();														//开始发送数据
	MyIIC_SendByteData(TH_ADDERSS| 0x00);								//发送要写入得地址
	if(MyIIC_Recv_ACK() != 0)													//从机应答
	{
		rt_kprintf("执行写入操作，TH未对其未应答！\n");
		return -1;
	}
	MyIIC_SendByteData(RegAddr);										//发送要写入的寄存器地址
	if(MyIIC_Recv_ACK() != 0)													//从机应答
	{
		rt_kprintf("选择对TH内寄存器地址操作失败，TH未应答！\n");
		return -1;
	}
	MyIIC_SendByteData(Data);											//发送要写入的数据
	if(MyIIC_Recv_ACK() != 0)
	{
		rt_kprintf("写入TH中该寄存器的数据有问题，TH无法应答！\n");
		return -1;
	}
	MyIIC_End();														//通讯结束
	//rt_kprintf("qq写入成功！\n");
	return 0;
}


//指定地址读（一个字节数据）
uint8_t TH_ReadByteData(uint8_t RegAddr)
{
	uint8_t RecValue;
	MyIIC_Start();														//开始发送数据
	MyIIC_SendByteData(TH_ADDERSS);								//发送要写入设备的地址
	if(MyIIC_Recv_ACK() != 0)													//从机应答
	{
		rt_kprintf("执行写入操作，TH未对其未应答！\n");
		return -1;
	}													
	MyIIC_SendByteData(RegAddr);										//发送要读出的寄存器地址
	if(MyIIC_Recv_ACK() != 0)													//从机应答
	{
		rt_kprintf("选择对TH内寄存器地址操作失败，TH未应答！\n");
		return -1;
	}
	
	MyIIC_Start();														//从新起始
	MyIIC_SendByteData(TH_ADDERSS|TH_I2C_DR);							//发送要读出设备的地址
	if(MyIIC_Recv_ACK() != 0)													//从机应答
	{
		rt_kprintf("执行读取操作，TH未对其未应答！\n");
		return -1;
	}
	RecValue = MyIIC_RecvByteData();									//将数据拿到
	MyIIC_Send_ACK(1);													//收一次数据 从机交回控制权
	MyIIC_End();														//通讯结束
	//rt_kprintf("读取成功！\n");
	return RecValue;
	
}


/*
功能：初始化温湿度传感器
*/
void InitTH(void)
{
	if(TH_WriteByteData(RESET_DRDY,0x70) == -1)
	{
		rt_kprintf("配置RESET_DRDY出错！\n");
	}
	if(TH_WriteByteData(MEASUREMENT,0x01) == -1)
	{
		rt_kprintf("配置MEASUREMENT出错！\n");
	}
}

/*
将温度湿度数据拿出
TH_Data:将拿到的温度湿度存储到其中，地址0为温度，1为湿度
返回值：
*/
char GetTHData(float *TH_Data)
{
	uint8_t TempH = 0,TempL = 0,HumiH = 0,HumiL = 0;
	float t = 0;
	TempL = TH_ReadByteData(TH_TEMPL);
	TempH = TH_ReadByteData(TH_TEMPH);
	t = (TempH<<8)|TempL;
	TH_Data[0] = ((t/65536)*165)-40;
	
	HumiL = TH_ReadByteData(TH_HUMIL);
	HumiH = TH_ReadByteData(TH_HUMIH);
	t = (HumiH<<8)|HumiL;
	TH_Data[1] = (t/65536)*100;
	return 0;
}


/*
打印出测得的温湿度数据

*/
void TH_PrintValue(float *TH_Data)
{
	rt_kprintf("温度：%.4f℃,湿度：%.4f%%\n",TH_Data[0],TH_Data[1]);
}


/*
功能：检测温湿度传感器的设备ID
返回值：
	返回设备的id号
*/
u16 TH_ID(void)
{
	u8 ID_H,ID_L;
	ID_H = TH_ReadByteData(0xFF);
	ID_L = TH_ReadByteData(0xFE);
	return (ID_H<<8)|ID_L;
}


static void test_thread_entry(void* parameter)
{	
  while (1)
  {
	GetTHData(THData);
	TH_PrintValue(THData);
    rt_thread_delay(2000);   /* 延时1000个tick */
  }
}



int bsp_TH_thread(void)
{
	test_thread =                          /* 线程控制块指针 */
    rt_thread_create( "test",              /* 线程名字 */
                      test_thread_entry,   /* 线程入口函数 */
                      RT_NULL,             /* 线程入口函数参数 */
                      512,                 /* 线程栈大小 */
                      3,                   /* 线程的优先级 */
                      20);                 /* 线程时间片 */
                   
    /* 启动线程，开启调度 */
   if (test_thread != RT_NULL)
        rt_thread_startup(test_thread);
    else
        return -1;
}