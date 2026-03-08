#include "SBUS_UART.h"
#include <string.h>
#include "main.h"
SBUS_CH_Struct SBUS_CH;
int command[20];			                                             //遥控器数据

//将sbus信号转化为通道值
uint8_t update_sbus(uint8_t *buf)
{
    int i;
    for (i=0;i<25;i++)
        SBUS_CH.signal[i] = buf[i];
		
		//rt_kprintf("%d \n",SBUS_CH.signal[23]);
	
	
    if(buf[23] == SBUS_CONNECT_FLAG)//遥控器连接上才进行数据转换
    {
			
		SBUS_CH.ConnectState = 1;
		SBUS_CH.CH1 =  ((int16_t)buf[ 1] >> 0 | ((int16_t)buf[ 2] << 8 )) & 0x07FF;
		SBUS_CH.CH2 =  ((int16_t)buf[ 2] >> 3 | ((int16_t)buf[ 3] << 5 )) & 0x07FF;
		SBUS_CH.CH3 =  ((int16_t)buf[ 3] >> 6 | ((int16_t)buf[ 4] << 2 ) | (int16_t)buf[ 5] << 10 ) & 0x07FF;
		SBUS_CH.CH4 =  ((int16_t)buf[ 5] >> 1 | ((int16_t)buf[ 6] << 7 )) & 0x07FF;
		SBUS_CH.CH5 =  ((int16_t)buf[ 6] >> 4 | ((int16_t)buf[ 7] << 4 )) & 0x07FF;
		SBUS_CH.CH6 =  ((int16_t)buf[ 7] >> 7 | ((int16_t)buf[ 8] << 1 ) | (int16_t)buf[9] << 9 ) & 0x07FF;
		SBUS_CH.CH7 =  ((int16_t)buf[ 9] >> 2 | ((int16_t)buf[10] << 6 )) & 0x07FF;
		SBUS_CH.CH8 =  ((int16_t)buf[10] >> 5 | ((int16_t)buf[11] << 3 )) & 0x07FF;
		SBUS_CH.CH9 =  ((int16_t)buf[12] << 0 | ((int16_t)buf[13] << 8 )) & 0x07FF;
		SBUS_CH.CH10 = ((int16_t)buf[13] >> 3 | ((int16_t)buf[14] << 5 )) & 0x07FF;
		SBUS_CH.CH11 = ((int16_t)buf[14] >> 6 | ((int16_t)buf[15] << 2 ) | (int16_t)buf[16] << 10 ) & 0x07FF;
		SBUS_CH.CH12 = ((int16_t)buf[16] >> 1 | ((int16_t)buf[17] << 7 )) & 0x07FF;
		SBUS_CH.CH13 = ((int16_t)buf[17] >> 4 | ((int16_t)buf[18] << 4 )) & 0x07FF;
		SBUS_CH.CH14 = ((int16_t)buf[18] >> 7 | ((int16_t)buf[19] << 1 ) | (int16_t)buf[20] << 9 ) & 0x07FF;
		SBUS_CH.CH15 = ((int16_t)buf[20] >> 2 | ((int16_t)buf[21] << 6 )) & 0x07FF;
		SBUS_CH.CH16 = ((int16_t)buf[21] >> 5 | ((int16_t)buf[22] << 3 )) & 0x07FF;
			
      return 1;

    }
    else 
    {
			SBUS_CH.ConnectState = 0;
			return 0;
    }
}


//将sbus信号通道值转化为特定区间的数值  [p_min,p_max] 
float sbus_to_Range(uint16_t sbus_value, float p_min, float p_max)
{
    float p;
    p = p_min + (float)(sbus_value - SBUS_RANGE_MIN) * (p_max-p_min)/(float)(SBUS_RANGE_MAX - SBUS_RANGE_MIN);  
    if (p > p_max) p = p_max;
    if (p < p_min) p = p_min;
    return p;
}



/**
  ******************************************************************************
  * @file    bsp_uart.c
  * @author  fire
  * @version V1.0
  * @date    
  * @brief   重定向c库printf函数到usart端口，中断接收模式
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */ 
 



rt_uint8_t USART6_Rx_Buf[BSP_USART6_RBUFF_SIZE];

///* 外部定义信号量控制块 */
//extern rt_sem_t USART6_sem;

 /**
  * @brief  配置嵌套向量中断控制器NVIC
  * @param  无
  * @retval 无
  */
static void NVIC_USART6_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 嵌套向量中断控制器组选择 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* 配置USART为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = BSP_USART6_IRQ;
  /* 抢断优先级为1 */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  /* 子优先级为1 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  /* 使能中断 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* 初始化配置NVIC */
  NVIC_Init(&NVIC_InitStructure);
}


 /**
  * @brief  BSP_USART6_Config GPIO 配置,工作模式配置。115200 8-N-1 ，中断接收模式
  * @param  无
  * @retval 无
  */
void BSP_USART6_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(BSP_USART6_RX_GPIO_CLK|BSP_USART6_TX_GPIO_CLK,ENABLE);

	/* 使能 USART 时钟 */
	RCC_APB2PeriphClockCmd(BSP_USART6_CLK, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART6时钟
												
	GPIO_PinAFConfig(BSP_USART6_RX_GPIO_PORT,BSP_USART6_RX_SOURCE,BSP_USART6_RX_AF);/*  连接 PXx 到 USARTx__Rx*/

	GPIO_PinAFConfig(BSP_USART6_TX_GPIO_PORT,BSP_USART6_TX_SOURCE,BSP_USART6_TX_AF);	 /* 连接 PXx 到 USARTx_Tx*/ 
	/* GPIO初始化 */
	GPIO_InitStructure.GPIO_Pin = BSP_USART6_TX_PIN|BSP_USART6_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BSP_USART6_RX_GPIO_PORT, &GPIO_InitStructure);
  
#ifdef	BSP_USART6_NORMAL_MODE

  /* 配置串BSP_USART6 模式 */
  /* 波特率设置：BSP_USART6_BAUDRATE */
  USART_InitStructure.USART_BaudRate = BSP_USART6_BAUDRATE;
  /* 字长(数据位+校验位)：8 */
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  /* 停止位：1个停止位 */
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* 校验位选择：不使用校验 */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  /* 硬件流控制：不使用硬件流 */
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  /* USART模式控制：同时使能接收和发送 */
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  /* 完成USART初始化配置 */
  USART_Init(BSP_USART6, &USART_InitStructure); 
	
  /* 嵌套向量中断控制器NVIC配置 */
	NVIC_USART6_Configuration();
  
	// 开启 串口空闲IDEL 中断
	USART_ITConfig(BSP_USART6, USART_IT_IDLE, ENABLE);  
  // 开启串口DMA接收
	USART_DMACmd(BSP_USART6, USART_DMAReq_Rx, ENABLE); 
	
	/* 使能串口 */
  USART_Cmd(BSP_USART6, ENABLE);
	
	#endif
	
	
	#ifdef BSP_USING_SBUS
	

	// 配置串口的工作参数
	// 配置波特率
	USART_InitStructure.USART_BaudRate = BSP_USART6_SBUS_BAUDRATE;									//100K
	// 配置 针数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// 配置停止位
	USART_InitStructure.USART_StopBits = USART_StopBits_2;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_Even ;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = 
	USART_HardwareFlowControl_None; 
	// 配置工作模式，收发一起
	USART_InitStructure.USART_Mode = USART_Mode_Rx |USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(BSP_USART6, &USART_InitStructure);
	// 串口中断优先级配置
	NVIC_USART6_Configuration();
	// 开启 串口空闲IDEL 中断
	USART_ITConfig(BSP_USART6, USART_IT_IDLE, ENABLE);  
	
	USART_ITConfig(BSP_USART6, USART_IT_RXNE, ENABLE);  
  
	// 使能串口
	USART_Cmd(BSP_USART6, ENABLE);	    
	
	#endif
}



//void BSP_USRT3_SEND_SEM(void)
//{
//	rt_sem_release(USART6_sem);  
//}


void BSP_USART6_DMA_Config(void)
{
  DMA_InitTypeDef DMA_InitStructure;

  // 开启DMA时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  // DMA复位
  DMA_DeInit(BSP_USART6_DMA_STREAM);  
  // 设置DMA通道
  DMA_InitStructure.DMA_Channel = BSP_USART6_RX_DMA_CHANNEL;  
  /*设置DMA源：串口数据寄存器地址*/
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)BSP_USART6_DR_ADDRESS;
  // 内存地址(要传输的变量的指针)
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART6_Rx_Buf;
  // 方向：从外设到内存
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  // 传输大小	
  DMA_InitStructure.DMA_BufferSize = BSP_USART6_RBUFF_SIZE;
  // 外设地址不增	    
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  // 内存地址自增
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  // 外设数据单位	
  DMA_InitStructure.DMA_PeripheralDataSize = 
  DMA_PeripheralDataSize_Byte;
  // 内存数据单位 一个数据大小
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 
  // DMA模式，一次或者循环模式
 // DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	
  // 优先级：非常高	
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; 
  // 禁止内存到内存的传输
  /*禁用FIFO*/
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;        
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;    
  /*存储器突发传输 1个节拍*/
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;    
  /*外设突发传输 1个节拍*/
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;    
  /*配置DMA2的数据流1*/		   
  DMA_Init(BSP_USART6_DMA_STREAM, &DMA_InitStructure);
  // 清除DMA所有标志
  DMA_ClearFlag(BSP_USART6_DMA_STREAM,DMA_FLAG_TCIF2);
  DMA_ITConfig(BSP_USART6_DMA_STREAM, DMA_IT_TE, ENABLE);
  // 使能DMA
  DMA_Cmd (BSP_USART6_DMA_STREAM,ENABLE);
}

void BSP_USART6_DMA_Rx_Data(void)
{
   
   DMA_Cmd(BSP_USART6_DMA_STREAM, DISABLE);     														// 关闭DMA ，防止干扰 
   DMA_ClearFlag(BSP_USART6_DMA_STREAM,DMA_FLAG_TCIF2);     								// 清DMA标志位    
   DMA_SetCurrDataCounter(BSP_USART6_DMA_STREAM,BSP_USART6_RBUFF_SIZE);     //  重新赋值计数值，必须大于等于最大可能接收到的数据帧数目
   DMA_Cmd(BSP_USART6_DMA_STREAM, ENABLE);       
   
   //rt_sem_release(USART6_sem);  //给出二值信号量 ，发送接收到新数据标志，供前台程序查询
  /* 
    DMA 开启，等待数据。注意，如果中断发送数据帧的速率很快，MCU来不及处理此次接收到的数据，
    中断又发来数据的话，这里不能开启，否则数据会被覆盖。有2种方式解决：

    1. 在重新开启接收DMA通道之前，将LumMod_Rx_Buf缓冲区里面的数据复制到另外一个数组中，
    然后再开启DMA，然后马上处理复制出来的数据。

    2. 建立双缓冲，在LumMod_Uart_DMA_Rx_Data函数中，重新配置DMA_MemoryBaseAddr 的缓冲区地址，
    那么下次接收到的数据就会保存到新的缓冲区中，不至于被覆盖。
  */
}


/*****************  发送一个字符 **********************/
void BSP_USART6_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* 发送一个字节数据到USART */
	USART_SendData(pUSARTx,ch);
	
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/*****************  发送字符串 **********************/
void BSP_USART6_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      BSP_USART6_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* 等待发送完成 */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}

/*****************  发送一个16位数 **********************/
void BSP_USART6_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
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




/*
******************************************************************
                              应用相关
******************************************************************
*/


#if 0

void remot_control_ratio_angel(double *a_angle,double *b_angle)
{
	double EQ1 = wheelbase_L/2;						//轴距一半	
	double EO = wheelbase_T/2;						//车宽一半
	double wheel_O_distance = 0;//车轮到车中心的直线距离
//  double a_ang = 0;
	double b_ang = 0;
	wheel_O_distance = pow(EQ1,2)+pow(EO,2);
	wheel_O_distance = sqrt(wheel_O_distance);
	
	 b_ang = asin(EO/wheel_O_distance)*180/PI_M;
	*b_angle = b_ang;
	
	b_ang = 90 - *b_angle;
	*a_angle = b_ang;
}




int control_chanl = 0;//遥控器控制的模式
void get_remot_sbus_control_device(void)
{
//	static rt_bool_t push_rod = RT_FALSE;//推杆是否伸出
//	static rt_uint32_t Pushrod_retraction = 0;//推杆收回时间
	robot_status.device_config.control_model = SBUS_CH.ConnectState;				//1，遥控；0，导航
	robot_status.device_config.run_model = 0;																//遥控器控制是行进模式 0：正常模式，1：平移，2：原地旋转，4：左斜移，8右斜移
//	robot_status.device_config.control_device = 0;													//遥控器控制是行进模式 0：不控任何设备，1：大灯，2：刹车，4：转向灯
#if 0
	if((SBUS_CH.CH7!=0))//旋转角度提取
	{
		if(abs(SBUS_CH.CH7-SBUS_MEDIAN_VALUE) > SBUS_DIFFER_VALUE)
		{
			if(SBUS_CH.CH7 > (SBUS_MEDIAN_VALUE + SBUS_SWITCH_VALUE))//左斜移
			{
				robot_status.device_config.run_model = 0x04;
				
				remot_control_angle_speed[0] = angle_change_pwm(45);
				remot_control_angle_speed[1] = angle_change_pwm(45);
				remot_control_angle_speed[2] = angle_change_pwm(45);
				remot_control_angle_speed[3] = angle_change_pwm(45);	
				
			}
			else if(SBUS_CH.CH7 < (SBUS_MEDIAN_VALUE - SBUS_SWITCH_VALUE))//右斜移
			{
				robot_status.device_config.run_model = 0x08;
				remot_control_angle_speed[0] = angle_change_pwm(0-45);
				remot_control_angle_speed[1] = angle_change_pwm(0-45);
				remot_control_angle_speed[2] = angle_change_pwm(0-45);
				remot_control_angle_speed[3] = angle_change_pwm(0-45);
			}
		}
	}
	if(SBUS_CH.CH6 > (SBUS_MEDIAN_VALUE + SBUS_SWITCH_VALUE))//平移
	{
		robot_status.device_config.run_model = 0x01;
		remot_control_angle_speed[0] = angle_change_pwm(-90);
		remot_control_angle_speed[1] = angle_change_pwm(-90);
		remot_control_angle_speed[2] = angle_change_pwm(-90);
		remot_control_angle_speed[3] = angle_change_pwm(-90);
	}

	if(SBUS_CH.CH8 > (SBUS_MEDIAN_VALUE + SBUS_SWITCH_VALUE))//原地旋转
	{
		double a_angle= 0;
		double b_angle = 0;
		robot_status.device_config.run_model = 0x02;
		remot_control_ratio_angel(&a_angle,&b_angle);
		remot_control_angle_speed[0] = angle_change_pwm(180-a_angle);
		remot_control_angle_speed[1] = angle_change_pwm(a_angle);
		remot_control_angle_speed[2] = angle_change_pwm((180-a_angle)*(-1));
		remot_control_angle_speed[3] = angle_change_pwm(-a_angle);
	}
	#endif
	
	
	if(SBUS_CH.CH8 > (SBUS_MEDIAN_VALUE + SBUS_SWITCH_VALUE))//原地旋转
	{
		control_chanl = 4;//原地旋转  方向旋转45
	}
	else
	{
		robot_status.device_config.remot_speed_model = 0;//低速
		if(SBUS_CH.CH8 < (SBUS_MEDIAN_VALUE - SBUS_SWITCH_VALUE))//高速
		{
			robot_status.device_config.remot_speed_model = 1;
		}
		
		control_chanl = 0;//原地旋转  方向旋转0
		if(SBUS_CH.CH6 > (SBUS_MEDIAN_VALUE + SBUS_SWITCH_VALUE))//平移
		{
			control_chanl = 1;//平移  方向旋转90
		}
		else if(SBUS_CH.CH7 > (SBUS_MEDIAN_VALUE + SBUS_SWITCH_VALUE))//左斜移
		{
			control_chanl = 2;//左前平移  方向旋转45
		}
		else if(SBUS_CH.CH7 < (SBUS_MEDIAN_VALUE - SBUS_SWITCH_VALUE))//右斜移
		{
			control_chanl = 3;//右前平移  方向旋转45
		}
		else
		{
			control_chanl = 0;//前平移  方向旋转0
		}
	}
	
	
	
	if(control_chanl == 1)//左右平移  方向旋转90
	{
		robot_status.device_config.run_model = 0x01;
		remot_control_angle_speed[0] = angle_change_pwm(-90);
		remot_control_angle_speed[1] = angle_change_pwm(-90);
		remot_control_angle_speed[2] = angle_change_pwm(-90);
		remot_control_angle_speed[3] = angle_change_pwm(-90);
		
	}
	else if(control_chanl ==2)//左前平移  方向旋转45
	{
		robot_status.device_config.run_model = 0x04;
		
		remot_control_angle_speed[0] = angle_change_pwm(44.8);
		remot_control_angle_speed[1] = angle_change_pwm(44.8);
		remot_control_angle_speed[2] = angle_change_pwm(44.8);
		remot_control_angle_speed[3] = angle_change_pwm(44.8);	
	}
	else if(control_chanl ==3)//右前平移  方向旋转45
	{
		robot_status.device_config.run_model = 0x08;
		remot_control_angle_speed[0] = angle_change_pwm(0-44.8);
		remot_control_angle_speed[1] = angle_change_pwm(0-44.8);
		remot_control_angle_speed[2] = angle_change_pwm(0-44.8);
		remot_control_angle_speed[3] = angle_change_pwm(0-44.8);
	}
	else if(control_chanl ==4)//原地旋转  方向旋转45
	{
		double a_angle= 0;
		double b_angle = 0;
		robot_status.device_config.run_model = 0x02;
		remot_control_ratio_angel(&a_angle,&b_angle);
		remot_control_angle_speed[0] = angle_change_pwm(-a_angle);
		remot_control_angle_speed[1] = angle_change_pwm(a_angle);
		remot_control_angle_speed[2] = angle_change_pwm(a_angle);
		remot_control_angle_speed[3] = angle_change_pwm(-a_angle);
	}
//	else if(control_chanl == 0)//前后平移
//	{
//		robot_status.device_config.run_model = 0x00;
//		remot_control_angle_speed[0] = angle_change_pwm(0);
//		remot_control_angle_speed[1] = angle_change_pwm(0);
//		remot_control_angle_speed[2] = angle_change_pwm(0);
//		remot_control_angle_speed[3] = angle_change_pwm(0);
//	}

	
	
	if(SBUS_CH.CH10 > (SBUS_MEDIAN_VALUE + SBUS_SWITCH_VALUE))//大灯
	{
		robot_status.device_config.control_device |= 0x01;
	}
	else
	{
		robot_status.device_config.control_device &= 0xfe;
	}
	if(SBUS_CH.CH9 > (SBUS_MEDIAN_VALUE + SBUS_SWITCH_VALUE))//刹车
	{
		robot_status.device_config.control_device |= 0x02;
	}
	else
	{
		robot_status.device_config.control_device &= 0xfd;
	}
	
	
	if(abs(SBUS_CH.CH5-SBUS_MEDIAN_VALUE) > SBUS_DIFFER_VALUE)//转向灯
	{
		if(SBUS_CH.CH5 > (SBUS_MEDIAN_VALUE + SBUS_SWITCH_VALUE))//左向灯
		{
			robot_status.device_config.control_device = 0x04;
			
		}
		else if(SBUS_CH.CH5 < (SBUS_MEDIAN_VALUE - SBUS_SWITCH_VALUE))//右向灯
		{
			robot_status.device_config.control_device = 0x08;
			
		}
	}
	else
	{
		robot_status.device_config.control_device &= 0xf3;
		
	}

	
	
	
//	if(SBUS_CH.CH10 > (SBUS_MEDIAN_VALUE + SBUS_SWITCH_VALUE))//大灯
//	{
//		robot_status.device_config.control_device |= 0x01;
//		BSP_OUT2_ON;
//	}
//	else
//	{
//		robot_status.device_config.control_device &= 0xfe;
//		BSP_OUT2_OFF;
//	}
//	if(SBUS_CH.CH9 > (SBUS_MEDIAN_VALUE + SBUS_SWITCH_VALUE))//刹车
//	{
//		robot_status.device_config.control_device |= 0x02;
//		Pushrod_retraction = 0;//推杆收回时间
//		if((robot_status.device_config.control_device & 0x20)||(robot_status.device_config.control_device & 0x40))
//		{
//			bsp_out_control &= 0xdf;
//			BSP_OUT6_OFF
//		}
//		else
//		{
//			if(push_rod == RT_FALSE)
//			{
//				push_rod = RT_TRUE;
//				BSP_OUT6_ON
//			}
//			
//			BSP_OUT7_OFF
//		}
//	}
//	else
//	{
//		push_rod = RT_FALSE;//推杆是否伸出
//		robot_status.device_config.control_device &= 0xfd;
//		Pushrod_retraction++;
//		BSP_OUT6_OFF
//		if(Pushrod_retraction >=300)
//		{
//			Pushrod_retraction = 300;
//			BSP_OUT7_OFF
//		}
//		else
//		{
//			BSP_OUT7_ON
//		}
//		
//	}
//	
//	
//	if(abs(SBUS_CH.CH5-SBUS_MEDIAN_VALUE) > SBUS_DIFFER_VALUE)//转向灯
//	{
//		if(SBUS_CH.CH5 > (SBUS_MEDIAN_VALUE + SBUS_SWITCH_VALUE))//左向灯
//		{
//			robot_status.device_config.control_device = 0x04;
//			BSP_OUT4_ON;
//			BSP_OUT5_OFF;
//		}
//		else if(SBUS_CH.CH5 < (SBUS_MEDIAN_VALUE - SBUS_SWITCH_VALUE))//右向灯
//		{
//			robot_status.device_config.control_device = 0x08;
//			BSP_OUT5_ON;
//			BSP_OUT4_OFF;
//		}
//	}
//	else
//	{
//		robot_status.device_config.control_device &= 0xf3;
//		BSP_OUT5_OFF;
//		BSP_OUT4_OFF;
//	}

	//bsp_out_control = robot_status.device_config.control_device;//控制输出
}

/**
 *弧度转化为角度
 * @param                
 */
extern double ArcToAngle(const double Arc);


/**
 *角度转化为弧度
 * @param                
 */

extern double AngleToArc(const double Angle);

/**
 *转速转换为轮子速度  60.0为60秒
 * @param                
 */
extern float rpm_change_vel(float rpm_val);
	

//底盘控制摇杆值转换速度
void coordinate_transformation_vx_vw(double x, double y,int *move_speed,int *rotate_angle,int driver_max_value)//坐标转换
{
	double Vf ;									//X,Y合成量
	static double Vf_angle ;						//X,Y合成量与Y轴角度，角速度
	double Vx;                	//线速度
  double Vf_angle_compare =0;						//X,Y合成量与Y轴角度，角速度
	Vf_angle = 0;
	Vf = 0;											//X,Y合成量
	Vx = 0;                	//线速度
	Vf = pow(x,2)+pow(y,2);
	Vf = sqrt(Vf);
	
	if(y > SBUS_DIFFER_VALUE*4)
	{
		Vx = y/SBUS_RANGE_VALUE*rpm_change_vel(motor_speed);//线速度
		
		Vf_angle_compare = arcsin(x/Vf)*(-1);//X,Y合成量与Y轴角度
		//if(fabs(Vf_angle - Vf_angle_compare) > 1 )
		{
			Vf_angle = Vf_angle_compare;
		}
		
	}
	else if(y < (0-SBUS_DIFFER_VALUE*4))
	{
		Vx = y/SBUS_RANGE_VALUE*rpm_change_vel(motor_speed);//线速度
		if(x>0)//SBUS_DIFFER_VALUE
		{
			Vf_angle_compare = arccos(y/Vf)-180;//X,Y合成量与Y轴角度
		}
		else if(x < 0)//(0-SBUS_DIFFER_VALUE)
		{
			Vf_angle_compare = 180 - arccos(y/Vf);//X,Y合成量与Y轴角度
		}
		else
		{
			Vf_angle_compare = 0;
		}
		
		//if(fabs(Vf_angle - Vf_angle_compare) > 1 )
		{
			Vf_angle = Vf_angle_compare;
		}
	}
	else
	{
//		if((x > SBUS_DIFFER_VALUE*4)||(x < (0-SBUS_DIFFER_VALUE*4)))
//		{
//			Vx = fabs(y)/SBUS_RANGE_VALUE*rpm_change_vel(motor_speed);//线速度
//		
//			Vf_angle_compare = arcsin(x/Vf)*(-1);//X,Y合成量与Y轴角度
//			//if(fabs(Vf_angle - Vf_angle_compare) > 1 )
//			{
//				Vf_angle = Vf_angle_compare;
//			}
//		}
//		
	}
	
	if(robot_status.device_config.remot_speed_model)//高速
	{	
		Vx *= 1;
		//Vf_angle *= 0.1;
	}
	else
	{
		Vx *= 0.5;
		//Vf_angle *= 0.1;
	}
	
	if(Vf_angle >= 20)
	{
		Vf_angle = 20;
	}
	else if(Vf_angle <= -20)
	{
		Vf_angle = -20;
	}
	
	//four_drive_four_revolutions_calculating(Vx,AngleToArc(Vf_angle),remot_control_speed,remot_control_angle_speed);
	Ackerman_Kinematics(Vf_angle,Vx,remot_control_speed,remot_control_angle_speed);
	
}


//coordinate_transformation_vx_vw(double x, double y,int *speed,int driver_max_value)//坐标转换
double left_speed = 0;
double right_speed = 0;
double Vx_speed = 0;
double Vw_speed = 0;
double Vw_speed_compare = 0;
double Vw_speed_limit = 2;

int remot_control_speed2[4] = {0,0};		//遥控器转换完成给到机器人的速度(轮子速度)
int remot_control_angle_speed2[4] = {0,0};		//遥控器转换完成给到机器人的速度(轮子角度)

void get_remot_sbus_control_speed(void)
{
//	int remot_speed[2] = {0,0};		//遥控器转换完成给到机器人的速度(轮子速度)
	static float speedx = 0;
	static float speedy = 0;
	#if 0
	if((SBUS_CH.CH1!=0) &&(SBUS_CH.CH2!=0))//行进速度提取
	{
		if((abs(SBUS_CH.CH1-SBUS_MEDIAN_VALUE) > SBUS_DIFFER_VALUE)||(abs(SBUS_CH.CH2-SBUS_MEDIAN_VALUE)>SBUS_DIFFER_VALUE))
		{
			speedx = (SBUS_CH.CH1-SBUS_MEDIAN_VALUE);
			speedy = (SBUS_CH.CH2-SBUS_MEDIAN_VALUE)*(-1);
			
			coordinate_transformation((double)speedx / SBUS_RANGE_VALUE , (double)speedy / SBUS_RANGE_VALUE,remot_control_speed,SPEED_MAX_48150_1E );//SBUS 协议  坐标转换

		}
		else
		{
			speedx = 0;
			speedy = 0;
			remot_control_speed[0] = 0;
			remot_control_speed[1] = 0;
		}
		
	}
	else
	{
		remot_control_speed[0] = 0;
		remot_control_speed[1] = 0;
		
	}
	
	#endif
	robot_status.device_status.obtacle_enble = 0;//
	if((abs(SBUS_CH.CH4-SBUS_MEDIAN_VALUE) > SBUS_DIFFER_VALUE*4))
	{
		if(SBUS_CH.CH4 > SBUS_MEDIAN_VALUE)
		{
			robot_status.device_status.obtacle_enble = 1;//禁用避障
		}
	}
	
	if(robot_status.device_config.run_model == 0)//遥控器自由控制模式
	{
		if((SBUS_CH.CH1!=0) &&(SBUS_CH.CH2!=0))//行进速度提取
		{
			if((abs(SBUS_CH.CH1-SBUS_MEDIAN_VALUE) > SBUS_DIFFER_VALUE)||(abs(SBUS_CH.CH2-SBUS_MEDIAN_VALUE)>SBUS_DIFFER_VALUE))
			{
				speedx = (SBUS_CH.CH1-SBUS_MEDIAN_VALUE);
				speedy = (SBUS_CH.CH2-SBUS_MEDIAN_VALUE)*(-1);
				
				coordinate_transformation_vx_vw(speedx,speedy,remot_control_speed,remot_control_angle_speed,SPEED_MAX_48150_1E);//坐标转换
					
//				coordinate_transformation((double)speedx / SBUS_RANGE_VALUE , (double)speedy / SBUS_RANGE_VALUE,remot_speed,SPEED_MAX_48150_1E );//SBUS 协议  坐标转换
//				left_speed = rpm_change_vel(remot_speed[0]) ;//左
//				right_speed = rpm_change_vel(remot_speed[1]) ;//右
//				Vx_speed = (left_speed+right_speed)/2;
//				
//				Vw_speed = ArcToAngle((right_speed - left_speed)/wheelbase_T);
//				if(fabs(Vw_speed - Vw_speed_compare) >= Vw_speed_limit)
//				{
//					Vw_speed_compare = Vw_speed;
//					
//				}
//				
//				four_drive_four_revolutions_calculating(Vx_speed,
//					AngleToArc(Vw_speed_compare),
//					remot_control_speed,
//					remot_control_angle_speed);		
				
			}
			else
			{
				speedx = 0;
				speedy = 0;
				Vw_speed_compare =0;
				remot_control_speed[0]  = 0 ;
				remot_control_speed[1]  = 0;
				remot_control_speed[2]  = 0;
				remot_control_speed[3]  = 0;

				remot_control_angle_speed[0]  = 0;
				remot_control_angle_speed[1]  = 0;
				remot_control_angle_speed[2]  = 0;
				remot_control_angle_speed[3]  = 0;
			}
			
		}
		else
		{
			Vw_speed_compare =0;
			remot_control_speed[0]  = 0 ;
			remot_control_speed[1]  = 0;
			remot_control_speed[2]  = 0;
			remot_control_speed[3]  = 0;

			remot_control_angle_speed[0]  = 0;
			remot_control_angle_speed[1]  = 0;
			remot_control_angle_speed[2]  = 0;
			remot_control_angle_speed[3]  = 0;
		
		}
	}
	else
	{
		if((SBUS_CH.CH2!=0))//行进速度提取
		{
			if((abs(SBUS_CH.CH2-SBUS_MEDIAN_VALUE)>SBUS_DIFFER_VALUE))
			{
				if(robot_status.device_config.control_device & 0x02)//刹车
				{
					speedx = 0;
					speedy = 0;
					remot_control_speed[0] = 0;
					remot_control_speed[1] = 0;
					remot_control_speed[2] = 0;
					remot_control_speed[3] = 0;
				}
				else
				{
					speedy = (SBUS_CH.CH2-SBUS_MEDIAN_VALUE)*(-1);
					remot_control_speed[0] = (double)speedy / SBUS_RANGE_VALUE * SPEED_MAX_48150_1E;
					remot_control_speed[1] = (double)speedy / SBUS_RANGE_VALUE * SPEED_MAX_48150_1E;
					remot_control_speed[2] = (double)speedy / SBUS_RANGE_VALUE * SPEED_MAX_48150_1E;
					remot_control_speed[3] = (double)speedy / SBUS_RANGE_VALUE * SPEED_MAX_48150_1E;
					if(robot_status.device_config.run_model & 0x02)//原地旋转
					{
						remot_control_speed[0] *= (-1);
						remot_control_speed[2] *= (-1);
					}
				}
				
				//coordinate_transformation((double)speedx / SBUS_RANGE_VALUE , (double)speedy / SBUS_RANGE_VALUE,remot_control_speed,SPEED_MAX_48150_1E );//SBUS 协议  坐标转换

			}
			else
			{
				speedx = 0;
				speedy = 0;
				remot_control_speed[0] = 0;
				remot_control_speed[1] = 0;
				remot_control_speed[2] = 0;
				remot_control_speed[3] = 0;
			}
			
		}
		else
		{
			remot_control_speed[0] = 0;
			remot_control_speed[1] = 0;
			remot_control_speed[2] = 0;
			remot_control_speed[3] = 0;
			
		}
	}
	
	
	
}



/*
******************************************************************
*                               变量
******************************************************************
*/

/* 定义线程控制块 */
static rt_thread_t USART6_thread = RT_NULL;
/* 定义信号量控制块 */
rt_sem_t USART6_sem = RT_NULL;


/*
*************************************************************************
*                             线程定义
*************************************************************************
*/

static void USART6_thread_entry(void* parameter)
{
  rt_err_t uwRet = RT_EOK;	

    /* 任务都是一个无限循环，不能返回 */
  while (1)
  {
		uwRet = rt_sem_take(USART6_sem,500);
    if(RT_EOK == uwRet)
    {
			
			get_remot_sbus_control_device();//获取遥控器设备控制
			get_remot_sbus_control_speed();//获取遥控器控制速度
			
    }
		else
		{
			remot_control_speed[0] = 0;
			remot_control_speed[1] = 0;
			robot_status.device_config.run_model = 0;				//遥控器控制是行进模式 0：正常模式，1：平移，2：原地旋转，4：左斜移，8右斜移
			//robot_status.device_config.control_device = 0;	//遥控器控制是行进模式 0：不控任何设备，1：大灯，2：刹车，4：转向灯
			robot_status.device_config.control_model =0;    //1，遥控；0，导航
		}
  }
}

int bsp_uart3_thread(void)
{

  /* 创建一个信号量 */
	USART6_sem = rt_sem_create("USART6_sem",/* 消息队列名字 */
                     0,     /* 信号量初始值，默认有一个信号量 */
                     RT_IPC_FLAG_FIFO); /* 信号量模式 FIFO(0x00)*/
  USART6_thread =                          /* 线程控制块指针 */
    rt_thread_create( "USART6",              /* 线程名字 */
                      USART6_thread_entry,   /* 线程入口函数 */
                      RT_NULL,             /* 线程入口函数参数 */
                      1024,                 /* 线程栈大小 */
                      3,                   /* 线程的优先级 */
                      20);                 /* 线程时间片 */
                   
    /* 启动线程，开启调度 */
   if (USART6_thread != RT_NULL)
        rt_thread_startup(USART6_thread);
    else
        return -1;
		
		return 0;

}

#endif


//static void SBUS_thread_entry(void *parameter)
//{
//	
//	double x = 0;
//	double y = 0;
//	
//	WheelEnableAll();
//	while(1)
//	{
//		x = (1024-SBUS_CH.CH2)/670.5;/*输入x轴的量程占比*/
//		y = (1024-SBUS_CH.CH1)/670.5;/*输入y轴的量程占比*/
//		coordinate_transformation(x,y,LRSpeed,506);/*将速度值计算好取出*/
//		CarRun(LRSpeed);/*输入相应的速度让电机转动*/
//		
////		rt_kprintf("LRSpeed[0] = %d LRSpeed[1] = %d\n",LRSpeed[0],LRSpeed[1]);
////		WheelSpeedSet(&SBUS_CH);
//		rt_thread_delay(1);
//	}
//}

///* 定义线程控制块 */
//static rt_thread_t SBUS_thread = RT_NULL;

//int bsp_SBUS_thread(void)
//{
//	//rt_kprintf("SBUS接收到的数据为：");
//	SBUS_thread =                          /* 线程控制块指针 */
//    rt_thread_create( "SBUS",              /* 线程名字 */
//                     SBUS_thread_entry,   /* 线程入口函数 */
//                      RT_NULL,             /* 线程入口函数参数 */
//                      512,                 /* 线程栈大小 */
//                      3,                   /* 线程的优先级 */
//                      20);                 /* 线程时间片 */
//                   
//    /* 启动线程，开启调度 */
//   if (SBUS_thread != RT_NULL)
//        rt_thread_startup(SBUS_thread);
//    else
//        return -1;   
//}














/*********************************************END OF FILE**********************/






