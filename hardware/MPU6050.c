
#include "MPU6050.h"

//六轴姿态传感器的数据
 float AccGyroValue[6] ={0};
/* 定义线程控制块 */
static rt_thread_t MPU6050_thread = RT_NULL;


#if 1

/*********************************************-软件IIC部分-*********************************************/


//指定地址写（一个字节数据）
char MPU6050_WriteByteData(uint8_t RegAddr,uint8_t Data)
{

	MyIIC_Start();														//开始发送数据
	MyIIC_SendByteData(MPU6050_ADDRESS);								//发送要写入得地址
	if(MyIIC_Recv_ACK() != 0)													//从机应答
	{
		rt_kprintf("执行写入操作，MPU-6050未对其应答！\n");
		rt_kprintf("地址为：%x\n",MPU6050_ADDRESS);
		return -1;
	}
	MyIIC_SendByteData(RegAddr);										//发送要写入的寄存器地址
	if(MyIIC_Recv_ACK() != 0)													//从机应答
	{
		rt_kprintf("选择对MPU-6050内寄存器地址操作失败，MPU-6050未应答！\n");
		return -1;
	}
	MyIIC_SendByteData(Data);											//发送要写入的数据
	if(MyIIC_Recv_ACK() != 0)
	{
		rt_kprintf("写入MPU-6050中该寄存器的数据有问题，MPU-6050无法应答！\n");
		return -1;
	}
	MyIIC_End();														//通讯结束
	//rt_kprintf("qq写入成功！\n");
}

//指定地址读（一个字节数据）
uint8_t MPU6050_ReadByteData(uint8_t RegAddr)
{
	uint8_t RecValue = 1;
	MyIIC_Start();														//开始发送数据
	MyIIC_SendByteData(MPU6050_ADDRESS);								//发送要写入设备的地址
	if(MyIIC_Recv_ACK() != 0)													//从机应答
	{
		rt_kprintf("执行读取操作，MPU-6050未对其未应答！\n");
		
		return -1;
	}													
	MyIIC_SendByteData(RegAddr);										//发送要读出的寄存器地址
	if(MyIIC_Recv_ACK() != 0)													//从机应答
	{
		rt_kprintf("选择对MPU-6050内寄存器地址操作失败，MPU-6050未应答！\n");
		return -1;
	}
	
	MyIIC_Start();														//从新起始
	MyIIC_SendByteData(MPU6050_ADDRESS|0x01);							//发送要读出设备的地址
	if(MyIIC_Recv_ACK() != 0)													//从机应答
	{
		rt_kprintf("执行读取操作，MPU-6050未对其未应答！\n");
		return -1;
	}
	RecValue = MyIIC_RecvByteData();									//将数据拿到
	MyIIC_Send_ACK(1);													//收一次数据 从机交回控制权
	MyIIC_End();														//通讯结束
	//rt_kprintf("读取成功！\n");
	return RecValue;
	
}




//初始化MPU6050为读取姿态传感器做准备
void Init_MPU6050(void)
{
	/*电源管理寄存器1，里面每位分别是 设备复位给0不复位，睡眠模式给0解除睡眠
	循环模式给0 不需要循环 无关位给零 温度传感器失能给0 不失能 最后三位选择时钟给0
	选择内部时钟，可以给001选择x轴陀螺仪时钟，该寄存器写入的数据就是0x01*/
	if(MPU6050_WriteByteData(MPU6050_PWR_MGMT_1,0x01) == -1)
	{
		rt_kprintf("写入MPU6050_PWR_MGMT_1寄存器失败！\n");
	}
	/*电源管理寄存器2，循环模式唤醒频率给00 不需要。后六位每个轴的待机位，全都给0不需要待机
	该寄存器写入的值是0x00*/
	if(MPU6050_WriteByteData(MPU6050_PWR_MGMT_2,0x00) == -1)
	{
		rt_kprintf("写入MPU6050_PWR_MGMT_2寄存器失败！\n");
	}
	
	/*采样率分频寄存器 这8位决定数据输出的快慢，值越小越快 给0x09 十分频*/
	
	if(MPU6050_WriteByteData(MPU6050_SMPLRT_DIV,0x09) == -1)
	{
		rt_kprintf("写入MPU6050_SMPLRT_DIV寄存器失败！\n");
	}
	/*配置寄存器 外部同步位给00 不需要，数字低通滤波器 根据需求来，给110 最平滑的滤波
	寄存器值为0x06*/
	
	if(MPU6050_WriteByteData(MPU6050_CONFIG,0x06) == -1)
	{
		rt_kprintf("写入MPU6050_CONFIG寄存器失败！\n");
	}
	/*陀螺仪配置寄存器 前面三位自测使能，不自测给000.满量程选择给11选择最大量程。后三位无关 */
	
	if(MPU6050_WriteByteData(MPU6050_GYRO_CONFIG,0x18) == -1)
	{
		rt_kprintf("写入MPU6050_GYRO_CONFIG寄存器失败！\n");
	}
	/*加速度器配置寄存器 自测给000  满量程给11  高通滤波器用不到 给000*/
	
	if(MPU6050_WriteByteData(MPU6050_ACCEL_CONFIG,0x18) == -1)
	{
		rt_kprintf("写入MPU6050_ACCEL_CONFIG寄存器失败！\n");
	}
}

/*
检测姿态设备是否连接上
返回值：0检测到设备，1未检测到设备
_Address：地址
*/
uint8_t MPU6050_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;
	MyIIC_Start();		/* 发送启动信号 */

	/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	MyIIC_SendByteData(_Address | 0x00);
	ucAck = MyIIC_Recv_ACK();	/* 检测设备的ACK应答 */

	MyIIC_End();			/* 发送停止信号 */

	return ucAck;
}

#endif


# if 0

/******************************************-硬件IIC部分-**********************************************/

void MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 10000;
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)
	{
		Timeout --;
		if (Timeout == 0)
		{
			break;
		}
	}
}	


//指定地址写（一个字节数据）
void MPU6050_WriteByteData(uint8_t RegAddr,uint8_t Data)
{
	I2C_GenerateSTART(I2C1, ENABLE);
	MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C1, RegAddr);
	MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	I2C_SendData(I2C1, Data);
	MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTOP(I2C1, ENABLE);
}

//指定地址读（一个字节数据）
uint8_t MPU6050_ReadByteData(uint8_t RegAddr)
{
	uint8_t Data;
	
	I2C_GenerateSTART(I2C1, ENABLE);
	MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C1, RegAddr);
	MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTART(I2C1, ENABLE);
	MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Receiver);
	MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
	
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	
	MPU6050_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data = I2C_ReceiveData(I2C1);
	
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	
	return Data;
	
}




//初始化MPU6050为读取姿态传感器做准备
void Init_MPU6050(void)
{
	Init_HardwareIIC();														//初始化IIC通讯
	/*电源管理寄存器1，里面每位分别是 设备复位给0不复位，睡眠模式给0解除睡眠
	循环模式给0 不需要循环 无关位给零 温度传感器失能给0 不失能 最后三位选择时钟给0
	选择内部时钟，可以给001选择x轴陀螺仪时钟，该寄存器写入的数据就是0x01*/
	MPU6050_WriteByteData(MPU6050_PWR_MGMT_1,0x01);							//
	/*电源管理寄存器2，循环模式唤醒频率给00 不需要。后六位每个轴的待机位，全都给0不需要待机
	该寄存器写入的值是0x00*/
	MPU6050_WriteByteData(MPU6050_PWR_MGMT_2,0x00);
	/*采样率分频寄存器 这8位决定数据输出的快慢，值越小越快 给0x09 十分频*/
	MPU6050_WriteByteData(MPU6050_SMPLRT_DIV,0x09);
	/*配置寄存器 外部同步位给00 不需要，数字低通滤波器 根据需求来，给110 最平滑的滤波
	寄存器值为0x06*/
	MPU6050_WriteByteData(MPU6050_CONFIG,0x06);
	/*陀螺仪配置寄存器 前面三位自测使能，不自测给000.满量程选择给11选择最大量程。后三位无关 */
	MPU6050_WriteByteData(MPU6050_GYRO_CONFIG,0x18);
	/*加速度器配置寄存器 自测给000  满量程给11  高通滤波器用不到 给000*/
	MPU6050_WriteByteData(MPU6050_ACCEL_CONFIG,0x18);
}
# endif
/******************************************-公共部分-**********************************************/


//将数据拿到出
void MPU6050_ReadData(float *AccGyroValue)
{
	short accx,accy,accz,gyrox,gyroy,gyroz;	
	float temp = 0.0;
	uint8_t DataH,DataL;
	DataH = MPU6050_ReadByteData(MPU6050_ACCEL_XOUT_H);
	DataL  = MPU6050_ReadByteData(MPU6050_ACCEL_XOUT_L);
	accx = (DataH<<8)|DataL;
	
	DataH = MPU6050_ReadByteData(MPU6050_ACCEL_YOUT_H);
	DataL  = MPU6050_ReadByteData(MPU6050_ACCEL_YOUT_L);
	accy = (DataH<<8)|DataL;

	DataH = MPU6050_ReadByteData(MPU6050_ACCEL_ZOUT_H);
	DataL  = MPU6050_ReadByteData(MPU6050_ACCEL_ZOUT_L);
	accz = (DataH<<8)|DataL;

	DataH = MPU6050_ReadByteData(MPU6050_GYRO_XOUT_H);
	DataL  = MPU6050_ReadByteData(MPU6050_GYRO_XOUT_L);
	AccGyroValue[3] = (DataH<<8)|DataL;

	DataH = MPU6050_ReadByteData(MPU6050_GYRO_YOUT_H);
	DataL  = MPU6050_ReadByteData(MPU6050_GYRO_YOUT_L);
	AccGyroValue[4] = (DataH<<8)|DataL;

	DataH = MPU6050_ReadByteData(MPU6050_GYRO_ZOUT_H);
	DataL  = MPU6050_ReadByteData(MPU6050_GYRO_ZOUT_L);
	AccGyroValue[5] = (DataH<<8)|DataL;
	
	//与自然x轴的的角度
	temp = accx/sqrt((accy*accy+accz*accz));
	temp = atan(temp);
	AccGyroValue[0] = 180/3.14*temp;
	
	//与自然y轴的的角度
	temp = accy/sqrt((accx*accx+accz*accz));
	temp = atan(temp);
	AccGyroValue[1] = 180/3.14*temp;
	
	//与自然z轴的的角度
	temp = sqrt((accy*accy+accx*accx))/accz;
	temp = atan(temp);
	AccGyroValue[2] = 180/3.14*temp;
}


static void MPU6050_thread_entry(float *parameter)
{
	while (1)
   {
		MPU6050_ReadData(parameter);    
		rt_kprintf("AccGyroValue[0] = %.3f\n",parameter[0]);
		rt_kprintf("AccGyroValue[1] = %.3f\n",parameter[1]);	
		rt_kprintf("AccGyroValue[2] = %.3f\n",parameter[2]);
		rt_kprintf("AccGyroValue[3] = %.3f\n",parameter[3]);
		rt_kprintf("AccGyroValue[4] = %.3f\n",parameter[4]);
		rt_kprintf("AccGyroValue[5] = %.3f\n",parameter[5]);
		//putchar(10);
		rt_thread_delay(2000);
	}
}

int bsp_MPU6050_thread(void)
{
	MPU6050_thread =                          /* 线程控制块指针 */
    rt_thread_create( "MPU6050",              /* 线程名字 */
                      MPU6050_thread_entry,   /* 线程入口函数 */
                      AccGyroValue,             /* 线程入口函数参数 */
                      512,                 /* 线程栈大小 */
                      3,                   /* 线程的优先级 */
                      20);                 /* 线程时间片 */
                   
    /* 启动线程，开启调度 */
   if (MPU6050_thread != RT_NULL)
        rt_thread_startup(MPU6050_thread);
    else
        return -1;   
}

