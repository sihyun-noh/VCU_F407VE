/***********************************************************************************************/
/*该功能为CAN的回环测试程序 使用CAN外设将数据从发送邮箱发送到自己的接收邮箱，接收部分通过接收中断
将标志位至1，主函数进行判断 若标志位为1则将接收到得数据打印到串口*/


#include "CAN.h"
#include "can2.h"

//初始化GPIO,配置CAN
void Init_CAN2_GPIO(void)
{
	//是能gpio口和can口的时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1|RCC_APB1Periph_CAN2,ENABLE);
	
	//讲CAN外设与gpio口复用
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_CAN2);
	
	//初始化gpio口
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	
	//初始化CAN
	CAN_InitTypeDef CAN_InitStructure;
	CAN_DeInit(CAN2);
	CAN_StructInit(&CAN_InitStructure);
	CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;									//配置CAN为回环模式
	CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq;										//配置BS1的长度
	CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;										//配置BS2的长度
	CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;										//配置SJW的极限值
	CAN_InitStructure.CAN_Prescaler = 42;											//配置CAN的分频值
	/*配置的can发送一位的频率为 42MHZ/42/(1+2+1) = 250KHZ */

	CAN_InitStructure.CAN_ABOM = ENABLE;											//是否使能ABOM自动离线管理功能
	CAN_InitStructure.CAN_AWUM = ENABLE;											//使能自动唤醒功能，会在监测到总线活动后自动唤醒
	CAN_InitStructure.CAN_NART = DISABLE;											//失能自动重传功能，若使能的话，如果发送不成功则会一直发送，失能则只发送一次
	CAN_InitStructure.CAN_RFLM = DISABLE;											//使能fifo锁定功能，若数据超出则将数据丢掉，若失能，输出超出会覆盖
	CAN_InitStructure.CAN_TTCM = DISABLE;											//失能时间触发功能
	CAN_InitStructure.CAN_TXFP = DISABLE;											//配置报文优先级，使能按照存入邮箱的先后顺序发送。失能按照ID优先级进行发送
	CAN_Init(CAN2,&CAN_InitStructure);
	
}

//初始化CAN的接收中断
void Init_CAN2_RecvIT(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;								//FIFO 0 用CAN2_RX0_IRQn
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

}

//初始化筛选器
void Init_CAN2_Filter(void)
{

	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;																//使能筛选器
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;												//将筛选器关联到CAN_Filter_FIFO0
	CAN_FilterInitStructure.CAN_FilterIdHigh = ((((u32)0x06000001<<3)|CAN_ID_EXT|CAN_RTR_Data)&0xffff0000)>>16;			//配置要筛选的数据，放在高位
	CAN_FilterInitStructure.CAN_FilterIdLow = (((u32)0x06000001<<3)|CAN_ID_EXT|CAN_RTR_Data)&0xffff;					//配置要筛选的数据，放在低位
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xffff;																//ID高位要和设定的一样
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xffff;																//ID低位要和设定的一样 两者加起来相当于工作在列表模式
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;														//工作在掩码模式
	CAN_FilterInitStructure.CAN_FilterNumber = 16;																		//使用第16组筛选器  CAN2只能使用后14组筛选器 14~27
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;													//位长位32位
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	//使能CAN的接收中断
	CAN_ITConfig(CAN2, CAN_IT_FMP0,ENABLE);																				//CAN_IT_FMP0表示当 FIFO0 接收到数据时会引起中断

}

//初始化CAN（总）
void Init_CAN2()
{
	Init_CAN2_GPIO();
	Init_CAN2_RecvIT();
	Init_CAN2_Filter();	
}

//设置发送数据
void CAN2_SetTransmit( CanTxMsg* TxMessage,char *data)
{
	u8 i = 0;
	for(i = 0;i<8;i++)
	{
		TxMessage->Data[i] = data[i];												//填充数据
	}
	
	TxMessage->DLC = 8;																//数据位长度
	TxMessage->ExtId = 0x06000001;													//扩展ID号
	TxMessage->IDE = CAN_Id_Extended;												//扩展ID模式
	TxMessage->RTR = CAN_RTR_Data;													//数据帧
	TxMessage->StdId = 0x00;														//标准ID号，在扩展ID模式下不需要填

}


//打印CAN接收的数据
void CAN2_PrintRecvData(CanRxMsg *RxMessage)
{
	u8 i = 0;
	for(i = 0;i<8;i++)
	{
		printf("%#04x  ",RxMessage->Data[i]);
	}
	putchar(10);
}

//打印CAN发送的数据
void CAN2_PrintSendData(CanTxMsg *TxMessage)
{
	u8 i = 0;
	for(i = 0;i<8;i++)
	{
		printf("%#04x  ",TxMessage->Data[i]);
	}
}













