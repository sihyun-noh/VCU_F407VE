/***********************************************************************************************/
/*该功能为CAN的回环测试程序 使用CAN外设将数据从发送邮箱发送到自己的接收邮箱，接收部分通过接收中断
将标志位至1，主函数进行判断 若标志位为1则将接收到得数据打印到串口*/

#include "CAN.h"

// can左右轮的速度
int LRSpeed[2] = { 0 };
extern SBUS_CH_Struct SBUS_CH;
// can的接收中断标志位
u8 CAN_Flag = 0;
// can的接收缓冲区和发送缓冲区
CanRxMsg RxMessage;
CanTxMsg TxMessage;
// can返回的发送邮箱的编号
u8 Mail_Box = 0;

// 初始化GPIO,配置CAN
void Init_CAN1_GPIO(void) {
  // 是能gpio口和can口的时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);

  // 将CAN外设与gpio口复用
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);

  // 初始化gpio口
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // 初始化CAN
  CAN_InitTypeDef CAN_InitStructure;
  CAN_DeInit(CAN1);
  CAN_StructInit(&CAN_InitStructure);
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; // 配置CAN为回环模式
  CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq;      // 配置BS1的长度
  CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;      // 配置BS2的长度
  CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;      // 配置SJW的极限值
	CAN_InitStructure.CAN_Prescaler = 21;         // 250khz
  /*配置的can发送一位的频率为 42MHZ/6/(1+2+4) = 1MHZ */

  CAN_InitStructure.CAN_ABOM = ENABLE;  // 是否使能ABOM自动离线管理功能
  CAN_InitStructure.CAN_AWUM = ENABLE;  // 使能自动唤醒功能，会在监测到总线活动后自动唤醒
  CAN_InitStructure.CAN_NART = DISABLE; // 失能自动重传功能，若使能的话，如果发送不成功则会一直发送，失能则只发送一次
  CAN_InitStructure.CAN_RFLM = DISABLE; // 使能fifo锁定功能，若数据超出则将数据丢掉，若失能，输出超出会覆盖
  CAN_InitStructure.CAN_TTCM = DISABLE; // 失能时间触发功能
  CAN_InitStructure.CAN_TXFP = DISABLE; // 配置报文优先级，使能按照存入邮箱的先后顺序发送。失能按照ID优先级进行发送
  CAN_Init(CAN1, &CAN_InitStructure);
}



void Init_CAN1_GPIO_AGMO(void) {
 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);

  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);


  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  CAN_InitTypeDef CAN_InitStructure;
  CAN_DeInit(CAN1);
  CAN_StructInit(&CAN_InitStructure);
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; 
  CAN_InitStructure.CAN_BS1 = CAN_BS1_11tq;     
  CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;      
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      
  CAN_InitStructure.CAN_Prescaler = 6;         // 500khz
	//CAN_InitStructure.CAN_Prescaler = 21;         // 250khz
  /*配置的can发送一位的频率为 42MHZ/6/(1+2+4) = 1MHZ */

  CAN_InitStructure.CAN_ABOM = ENABLE;  
  CAN_InitStructure.CAN_AWUM = ENABLE;  
  CAN_InitStructure.CAN_NART = DISABLE; 
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TTCM = DISABLE; 
  CAN_InitStructure.CAN_TXFP = DISABLE; 
  CAN_Init(CAN1, &CAN_InitStructure);
}


// 初始化CAN的接收中断
void Init_CAN1_RecvIT(void) {
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn; // FIFO 0 用CAN1_RX0_IRQn
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
}

// 初始化筛选器
void Init_CAN1_Filter(void) {
  CAN_FilterInitTypeDef CAN_FilterInitStructure;

  // 双电机驱动器
  CAN_FilterInitStructure.CAN_FilterNumber = 0; // 使用第0组筛选器
                                                // CAN1只能用前14组筛选器也就是0~13
  CAN_FilterInitStructure.CAN_FilterIdHigh =
      (((WheelLeftStdId << 3) | CAN_ID_EXT | CAN_RTR_Data) & 0xffff0000) >> 16; // 配置要筛选的数据，放在高位
  CAN_FilterInitStructure.CAN_FilterIdLow =
      ((WheelRightStdId << 3) | CAN_ID_EXT | CAN_RTR_Data) & 0xffff; // 配置要筛选的数据，放在低位
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xffff;             // ID高位要和设定的一样
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xffff;           // ID低位要和设定的一样 两者加起来相当于工作在列表模式
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; // 工作在掩码模式
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;     // 位长为32位
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; // 将筛选器关联到CAN_Filter_FIFO0
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;               // 使能筛选器
  CAN_FilterInit(&CAN_FilterInitStructure);

  // 四电机驱动器
  CAN_FilterInitStructure.CAN_FilterNumber = 1; // 使用第1组筛选器
                                                // CAN1只能用前14组筛选器也就是0~13
  CAN_FilterInitStructure.CAN_FilterIdHigh =
      ((0x601 << 3) | CAN_ID_EXT | CAN_RTR_Data) & 0xffff; // 配置要筛选的第一个16位数据
  CAN_FilterInitStructure.CAN_FilterIdLow =
      ((0x602 << 3) | CAN_ID_EXT | CAN_RTR_Data) & 0xffff; // 配置要筛选的第2个16位数据
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh =
      ((0x603 << 3) | CAN_ID_EXT | CAN_RTR_Data) & 0xffff; // 配置要筛选的第3个16位数据
  CAN_FilterInitStructure.CAN_FilterMaskIdLow =
      ((0x604 << 3) | CAN_ID_EXT | CAN_RTR_Data) & 0xffff;             // 配置要筛选的第4个16位数据
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;      // 工作在白名单模式
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;     // 位长为16位
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; // 将筛选器关联到CAN_Filter_FIFO0
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;               // 使能筛选器
  CAN_FilterInit(&CAN_FilterInitStructure);
  // 使能CAN的接收中断
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE); // CAN_IT_FMP0表示当 FIFO0 接收到数据时会引起中断
}


void Init_CAN1_Filter_AGMO(void) {
  // upper cmd
	uint32_t id_1 = (0x18ff0200 << 3) | CAN_ID_EXT | CAN_RTR_Data;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;

  CAN_FilterInitStructure.CAN_FilterNumber = 0;
                                                
  CAN_FilterInitStructure.CAN_FilterIdHigh = (id_1 >> 16) & 0xffff; 
  CAN_FilterInitStructure.CAN_FilterIdLow =  (id_1      ) & 0xffff;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xffff;            
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xffff;          
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; 
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;     
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; 
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;              
  CAN_FilterInit(&CAN_FilterInitStructure);
	
	 // upper rpm cmd
	uint32_t id_2 = (0x18ff0200 << 3) | CAN_ID_EXT | CAN_RTR_Data;

  CAN_FilterInitStructure.CAN_FilterNumber = 1;
                                                
  CAN_FilterInitStructure.CAN_FilterIdHigh = (id_2 >> 16) & 0xffff; 
  CAN_FilterInitStructure.CAN_FilterIdLow =  (id_2      ) & 0xffff;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xffff;            
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xffff;          
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; 
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;     
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; 
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;              
  CAN_FilterInit(&CAN_FilterInitStructure);

	// motor driver 1 status 
	uint32_t id_3 = (0x18ff0021 << 3) | CAN_ID_EXT | CAN_RTR_Data;


  CAN_FilterInitStructure.CAN_FilterNumber = 2;
                                                
  CAN_FilterInitStructure.CAN_FilterIdHigh = (id_3 >> 16) & 0xffff; 
  CAN_FilterInitStructure.CAN_FilterIdLow =  (id_3      ) & 0xffff;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xffff;            
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xffff;          
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; 
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;     
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; 
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;              
  CAN_FilterInit(&CAN_FilterInitStructure);
	
	// motor driver 2 status 
	uint32_t id_4 = (0x18ff0031 << 3) | CAN_ID_EXT | CAN_RTR_Data;

  CAN_FilterInitStructure.CAN_FilterNumber = 3;
                                                
  CAN_FilterInitStructure.CAN_FilterIdHigh = (id_4 >> 16) & 0xffff; 
  CAN_FilterInitStructure.CAN_FilterIdLow =  (id_4      ) & 0xffff;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xffff;            
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xffff;          
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; 
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;     
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; 
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;              
  CAN_FilterInit(&CAN_FilterInitStructure);
  
	
	// example 16bit
  CAN_FilterInitStructure.CAN_FilterNumber = 4; 
                                               
  CAN_FilterInitStructure.CAN_FilterIdHigh =
      ((0x601 << 3) | CAN_ID_EXT | CAN_RTR_Data) & 0xffff; 
  CAN_FilterInitStructure.CAN_FilterIdLow =
      ((0x602 << 3) | CAN_ID_EXT | CAN_RTR_Data) & 0xffff; 
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh =
      ((0x603 << 3) | CAN_ID_EXT | CAN_RTR_Data) & 0xffff; 
  CAN_FilterInitStructure.CAN_FilterMaskIdLow =
      ((0x604 << 3) | CAN_ID_EXT | CAN_RTR_Data) & 0xffff;             
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;      
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;    
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; 
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;              
  CAN_FilterInit(&CAN_FilterInitStructure);
  
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE); 
}

// 初始化CAN（总）
void Init_CAN1() {
  //Init_CAN1_GPIO();
	Init_CAN1_GPIO_AGMO();
  Init_CAN1_RecvIT();
  //Init_CAN1_Filter();
	Init_CAN1_Filter_AGMO();
}

/*
功能：设置要发送的数据。
参数：
    TxMessage：配置can发送信息的结构体变量，在里面存放要发送的数据。
        data： 里面存放要发送的数据。
        rtr：  设置为远程帧还是数据帧 CAN_RTR_Remote or CAN_RTR_Data。
        ide：  设置为扩展帧还是标准帧 CAN_Id_Standard or CAN_Id_Extended。
        std_id:标准ID号。
        ext_id:扩展ID号。
        dlc：  发动的数据信息长度，一般是8。
*/
void CAN1_SetTransmit(CanTxMsg* TxMessage, const char* data, u32 rtr, u32 ide, u16 std_id, u32 ext_id, u8 dlc) {
  u8 i = 0;
  for (i = 0; i < 8; i++) {
    TxMessage->Data[i] = data[i]; // 填充数据
  }

  TxMessage->DLC = dlc;      // 数据位长度
  TxMessage->ExtId = ext_id; // 扩展ID号
  TxMessage->IDE = ide;      // 扩展ID模式
  TxMessage->RTR = rtr;      // 数据帧
  TxMessage->StdId = std_id; // 标准ID号，在扩展ID模式下不需要填
}

/*
功能：打印can接收到的数据。
参数：
    RxMessage：can接收信息的结构体变量，里面有存放的接收到的信息。

*/
void CAN1_PrintRecvData(CanRxMsg* RxMessage) {
  u8 i = 0;
  for (i = 0; i < 8; i++) {
    rt_kprintf("%#04x  ", RxMessage->Data[i]);
  }
  putchar(10);
}

/*
功能：打印can要发送的数据。
参数：
    TxMessage：配置can发送信息的结构体变量，里面有存放的发送信息的数据。

*/
void CAN1_PrintSendData(CanTxMsg* TxMessage) {
  u8 i = 0;
  for (i = 0; i < 8; i++) {
    rt_kprintf("%#04x  ", TxMessage->Data[i]);
  }
}

/*------------------------------------------------keya电机控制器-------------------------------------------------------*/
// can的发送数据
const char Start_Data[8] = { 0x23, 0x0D, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00 };
const char Disable_Data[8] = { 0x23, 0x0C, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00 };
const char Speed_Data[8] = { 0x23, 0x00, 0x20, 0x01, 0x00, 0x00, 0x13, 0x88 };
const char Torque_Data[8] = { 0x23, 0x01, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00 };
const char Search_Value[8] = { 0x40, 0x04, 0x21, 0x02, 0x00, 0x00, 0x00, 0x00 };

/*
功能：给电机使能。
*/
void MotorEnable(void) {

  CAN1_SetTransmit(&TxMessage, Start_Data, Motor1RTR, Motor1IDE, Motor1StdId, Motor1ExtId, Motor1DLC);

  // 把放入数据的发送邮箱号返回出来，若该邮箱满会返回 CAN_TxStatus_NoMailBox
  Mail_Box = CAN_Transmit(CAN1, &TxMessage); // 将数据发送

  // 如果邮箱号存在
  if (Mail_Box != CAN_TxStatus_NoMailBox) {
    // 等待直到该邮箱将数据发送到CAN收发器完成
    while (CAN_TransmitStatus(CAN1, Mail_Box) != CAN_TxStatus_Ok)
      ; // 等待CAN收发器发送到总线完毕，可使用CAN_TransmitStatus查看状态
  } else {
    rt_kprintf("no mail_box !\n");
  }
  //	rt_kprintf("填充的CAN1数据为：");
  //	CAN1_PrintSendData(&TxMessage);
  //	putchar(10);
}

/*
功能：电机速度设置。
参数：
    speed：给定电机的速度，速度区间是0—255 0~127为反转，0为最大速度，127为最小速度
            129—255为正转，129为最小速度，255为最大速度。
*/
void MotorSpeedSet(u16 speed) {
  int temp = 0;
  char data[8] = { 0 };
  for (int i = 0; i < 8; i++) {
    data[i] = Speed_Data[i];
  }
  if (speed == 128) {
    CAN1_SetTransmit(&TxMessage, Disable_Data, Motor1RTR, Motor1IDE, Motor1StdId, Motor1ExtId, Motor1DLC);

    // 把放入数据的发送邮箱号返回出来，若该邮箱满会返回 CAN_TxStatus_NoMailBox
    Mail_Box = CAN_Transmit(CAN1, &TxMessage); // 将数据发送

    // 如果邮箱号存在
    if (Mail_Box != CAN_TxStatus_NoMailBox) {
      // 等待直到该邮箱将数据发送到CAN收发器完成
      while (CAN_TransmitStatus(CAN1, Mail_Box) != CAN_TxStatus_Ok)
        ; // 等待CAN收发器发送到总线完毕，可使用CAN_TransmitStatus查看状态

    } else {
      rt_kprintf("no mail_box !\n");
    }
    rt_kprintf("填充的CAN1数据为：");
    CAN1_PrintSendData(&TxMessage);
    putchar(10);
  } else if (speed >= 0 && speed <= 127) {
    temp = (speed - 128) * 78.125;
    data[4] = (temp >> 24);
    data[5] = (temp >> 16);
    data[6] = (temp >> 8);
    data[7] = (temp);

    CAN1_SetTransmit(&TxMessage, data, Motor1RTR, Motor1IDE, Motor1StdId, Motor1ExtId, Motor1DLC);

    // 把放入数据的发送邮箱号返回出来，若该邮箱满会返回 CAN_TxStatus_NoMailBox
    Mail_Box = CAN_Transmit(CAN1, &TxMessage); // 将数据发送

    // 如果邮箱号存在
    if (Mail_Box != CAN_TxStatus_NoMailBox) {
      // 等待直到该邮箱将数据发送到CAN收发器完成
      while (CAN_TransmitStatus(CAN1, Mail_Box) != CAN_TxStatus_Ok)
        ; // 等待CAN收发器发送到总线完毕，可使用CAN_TransmitStatus查看状态

    } else {
      rt_kprintf("no mail_box !\n");
    }
    rt_kprintf("填充的CAN1数据为：");
    CAN1_PrintSendData(&TxMessage);
    putchar(10);
  } else if (speed >= 129 && speed <= 255) {
    temp = (speed - 128) * 78.74;
    data[4] = (temp >> 24);
    data[5] = (temp >> 16);
    data[6] = (temp >> 8);
    data[7] = (temp);

    CAN1_SetTransmit(&TxMessage, data, Motor1RTR, Motor1IDE, Motor1StdId, Motor1ExtId, Motor1DLC);

    // 把放入数据的发送邮箱号返回出来，若该邮箱满会返回 CAN_TxStatus_NoMailBox
    Mail_Box = CAN_Transmit(CAN1, &TxMessage); // 将数据发送

    // 如果邮箱号存在
    if (Mail_Box != CAN_TxStatus_NoMailBox) {
      // 等待直到该邮箱将数据发送到CAN收发器完成
      while (CAN_TransmitStatus(CAN1, Mail_Box) != CAN_TxStatus_Ok)
        ; // 等待CAN收发器发送到总线完毕，可使用CAN_TransmitStatus查看状态

    } else {
      rt_kprintf("no mail_box !\n");
    }
    rt_kprintf("填充的CAN1数据为：");
    CAN1_PrintSendData(&TxMessage);
    putchar(10);
  } else {
    rt_kprintf("输入电机速度数据错误！\n");
  }
}

/*
功能：电机控制直接调用的函数，由使能函数和速度设置函数组成
参数：
    speed：输入电机的速度，与速度设置函数的参数一样
*/
void MotorSpeedControl(u16 speed) {
  MotorEnable();
  MotorSpeedSet(speed);
}

#if 0 
/*
功能：电机转矩设置。
参数：
	torque：给定电机的转矩，速度区间是0—255 0~127为反转，0为最大速度，127为最小速度
			129—255为正转，129为最小速度，255为最大速度。
*/
void MotorTorqueSet(u16 torque)
{
	int temp = 0;
	char data[8] = {0};
	for(int i = 0;i<8;i++)
	{
		data[i] = Torque_Data[i];
	}
	if(torque == 128)
	{
		CAN1_SetTransmit(&TxMessage,Disable_Data,Motor1RTR,Motor1IDE,Motor1StdId,Motor1ExtId,Motor1DLC);
	
		//把放入数据的发送邮箱号返回出来，若该邮箱满会返回 CAN_TxStatus_NoMailBox
		Mail_Box = CAN_Transmit(CAN1,&TxMessage);											//将数据发送

		//如果邮箱号存在
		if(Mail_Box != CAN_TxStatus_NoMailBox)
		{
			//等待直到该邮箱将数据发送到CAN收发器完成
			while(CAN_TransmitStatus(CAN1,Mail_Box)!=CAN_TxStatus_Ok );//等待CAN收发器发送到总线完毕，可使用CAN_TransmitStatus查看状态
			
		}
		else
		{
			rt_kprintf("no mail_box !\n");
		}
		rt_kprintf("填充的CAN1数据为：");
		CAN1_PrintSendData(&TxMessage);
		putchar(10);
	}
	else if(torque >=0 && torque <= 127)
	{
		temp = (torque -128)*78.125;
		data[4] = (temp>>24);
		data[5]	= (temp>>16);
		data[6] = (temp>>8);
		data[7] = (temp);

		CAN1_SetTransmit(&TxMessage,data,Motor1RTR,Motor1IDE,Motor1StdId,Motor1ExtId,Motor1DLC);
	
		//把放入数据的发送邮箱号返回出来，若该邮箱满会返回 CAN_TxStatus_NoMailBox
		Mail_Box = CAN_Transmit(CAN1,&TxMessage);											//将数据发送

		//如果邮箱号存在
		if(Mail_Box != CAN_TxStatus_NoMailBox)
		{
			//等待直到该邮箱将数据发送到CAN收发器完成
			while(CAN_TransmitStatus(CAN1,Mail_Box)!=CAN_TxStatus_Ok );//等待CAN收发器发送到总线完毕，可使用CAN_TransmitStatus查看状态
			
		}
		else
		{
			rt_kprintf("no mail_box !\n");
		}
		rt_kprintf("填充的CAN1数据为：");
		CAN1_PrintSendData(&TxMessage);
		putchar(10);
	}
	else if(torque >=129 && torque <= 255)
	{
		temp = (torque -128)*78.74;
		data[4] = (temp>>24);
		data[5]	= (temp>>16);
		data[6] = (temp>>8);
		data[7] = (temp);
		
		CAN1_SetTransmit(&TxMessage,data,Motor1RTR,Motor1IDE,Motor1StdId,Motor1ExtId,Motor1DLC);
	
		//把放入数据的发送邮箱号返回出来，若该邮箱满会返回 CAN_TxStatus_NoMailBox
		Mail_Box = CAN_Transmit(CAN1,&TxMessage);											//将数据发送

		//如果邮箱号存在
		if(Mail_Box != CAN_TxStatus_NoMailBox)
		{
			//等待直到该邮箱将数据发送到CAN收发器完成
			while(CAN_TransmitStatus(CAN1,Mail_Box)!=CAN_TxStatus_Ok );//等待CAN收发器发送到总线完毕，可使用CAN_TransmitStatus查看状态
			
		}
		else
		{
			rt_kprintf("no mail_box !\n");
		}
		rt_kprintf("填充的CAN1数据为：");
		CAN1_PrintSendData(&TxMessage);
		putchar(10);
	}
	else
	{
		rt_kprintf("输入电机速度数据错误！\n");
	}
}



/*
功能：电机扭矩控制直接调用的函数，由使能函数和速度设置函数组成
参数：
	speed：输入电机的速度，与速度设置函数的参数一样
*/
void MotorTorqueControl(u16 torque)
{
	MotorEnable();
	MotorTorqueSet(torque);
}

#endif

/*
功能：
*/
static void motor1_thread_entry(void* parameter) {
  u16* Speed = (u16*)parameter;
  u16 temp = *Speed;
  while (1) {

    MotorSpeedControl(temp);

    rt_thread_delay(500);
  }
}

/* 定义线程控制块 */
static rt_thread_t motor_thread = RT_NULL;

int bsp_motor1_thread(u16 Speed) {
  motor_thread =                            /* 线程控制块指针 */
      rt_thread_create("motor1",            /* 线程名字 */
                       motor1_thread_entry, /* 线程入口函数 */
                       &Speed,              /* 线程入口函数参数 */
                       512,                 /* 线程栈大小 */
                       3,                   /* 线程的优先级 */
                       20);                 /* 线程时间片 */

  /* 启动线程，开启调度 */
  if (motor_thread != RT_NULL)
    rt_thread_startup(motor_thread);
  else
    return -1;
}
/*-------------------------------------------------------------阿波罗小车电机控制器--------------------------------------------------------------*/

const char WheelData[][8] = {
  { 0x00, 0xFA, 0x00, 0x19, 0x00, 0x00, 0x00, 0x2F }, // 0 配置为速度模式
  { 0x00, 0xFA, 0x00, 0x13, 0x00, 0x00, 0x0A, 0x0B }, // 1 配置加速度时间为11*100ms
  { 0x00, 0xFA, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00 }, // 2 设置初始化目标速度为0
  { 0x00, 0xFA, 0x00, 0x10, 0x00, 0x00, 0x00, 0x1F }, // 3 使能电机
  { 0x00, 0xFA, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0F }, // 4 单个停止电机
  { 0x00, 0xFA, 0x00, 0x11, 0x00, 0x00, 0x01, 0xFA }, // 5 左轮前右轮后转最大（可设置）速度
  { 0x00, 0xFA, 0x00, 0x11, 0xFF, 0xFF, 0xFE, 0x06 }, // 6 右轮前左轮后转最大（可设置）速度
  { 0x00, 0xDA, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0F }, // 7 全部停止电机
  { 0x23, 0x00, 0x20, 0x01, 0xFA, 0x01, 0x00,
    0x00 }, // 8 DBL4875-2e四轮电机右轮最大速度506（-1000~1000）测试用   	右前轮正转用正值
  { 0x23, 0x00, 0x20, 0x02, 0xFA, 0x01, 0x00, 0x00 }, // 9 左轮最大速度506
                                                      // 左前轮正转用正值
  { 0x23, 0x00, 0x20, 0x01, 0x06, 0xFE, 0x00,
    0x00 }, // 10 DBL4875-2e四轮电机右轮最大速度-506（-1000~1000）测试用 	右后轮正转用负值
  { 0x23, 0x00, 0x20, 0x02, 0x06, 0xFE, 0x00, 0x00 }, // 11 左轮最大速度-506
                                                      // 左后轮正转用负值
  { 0x2F, 0x0C, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 12 DBL4875-2e紧急停止
  { 0x2F, 0x0D, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 13 DBL4875-2e紧急停止释放
  { 0x23, 0x00, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00 }, // 14 右轮0速
  { 0x23, 0x00, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00 }, // 15 左轮0速
};

const char WheelData_agmo[][8] = {
  { 0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 0 配置为速度模式
  { 0x00, 0xFA, 0x00, 0x13, 0x00, 0x00, 0x0A, 0x0B }, // 1 配置加速度时间为11*100ms
  { 0x00, 0xFA, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00 }, // 2 设置初始化目标速度为0
  { 0x00, 0xFA, 0x00, 0x10, 0x00, 0x00, 0x00, 0x1F }, // 3 使能电机
  { 0x00, 0xFA, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0F }, // 4 单个停止电机
  { 0x00, 0xFA, 0x00, 0x11, 0x00, 0x00, 0x01, 0xFA }, // 5 左轮前右轮后转最大（可设置）速度
  { 0x00, 0xFA, 0x00, 0x11, 0xFF, 0xFF, 0xFE, 0x06 }, // 6 右轮前左轮后转最大（可设置）速度
  { 0x00, 0xDA, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0F }, // 7 全部停止电机
  { 0x23, 0x00, 0x20, 0x01, 0xFA, 0x01, 0x00,
    0x00 }, // 8 DBL4875-2e四轮电机右轮最大速度506（-1000~1000）测试用   	右前轮正转用正值
  { 0x23, 0x00, 0x20, 0x02, 0xFA, 0x01, 0x00, 0x00 }, // 9 左轮最大速度506
                                                      // 左前轮正转用正值
  { 0x23, 0x00, 0x20, 0x01, 0x06, 0xFE, 0x00,
    0x00 }, // 10 DBL4875-2e四轮电机右轮最大速度-506（-1000~1000）测试用 	右后轮正转用负值
  { 0x23, 0x00, 0x20, 0x02, 0x06, 0xFE, 0x00, 0x00 }, // 11 左轮最大速度-506
                                                      // 左后轮正转用负值
  { 0x2F, 0x0C, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 12 DBL4875-2e紧急停止
  { 0x2F, 0x0D, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 13 DBL4875-2e紧急停止释放
  { 0x23, 0x00, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00 }, // 14 右轮0速
  { 0x23, 0x00, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00 }, // 15 左轮0速
};

// 发送can数据包
// modidfy
u8 SendCanDataPage(CanTxMsg* TxMessage, const char* data, u32 rtr, u32 ide, u16 std_id, u32 ext_id, u8 dlc) {
  int success_cut = 0;
  CAN1_SetTransmit(TxMessage, data, rtr, ide, std_id, ext_id, dlc);
  // 把放入数据的发送邮箱号返回出来，若该邮箱满会返回 CAN_TxStatus_NoMailBox
  Mail_Box = CAN_Transmit(CAN1, TxMessage); // 将数据发送

  // 如果邮箱号存在
  if (Mail_Box != CAN_TxStatus_NoMailBox) {

    // 等待直到该邮箱将数据发送到CAN收发器完成
    while (CAN_TransmitStatus(CAN1, Mail_Box) != CAN_TxStatus_Ok) {
      success_cut++;
      rt_thread_delay(100);
      if (success_cut > 10)
        break;
      rt_kprintf("CAN1_Transmit Statues error! \n");
    }; // 等待CAN收发器发送到总线完毕，可使用CAN_TransmitStatus查看状态

  } else {
    rt_kprintf("no mail_box !\n");
  }
  Mail_Box = 0;
}

/*
功能：给电机使能,可分别配置左右轮。
参数：
    num：可输入1或2，设置使能左轮还是右轮
*/
int8_t WheelEnable(u8 num) {

  // 左轮电机使能
  if (num == 2) {
    SendCanDataPage(&TxMessage, WheelData[0], WheelRTR, WheelIDE, WheelLeftStdId, WheelLeftExtId, WheelDLC);
    //		rt_thread_delay(50);

    SendCanDataPage(&TxMessage, WheelData[1], WheelRTR, WheelIDE, WheelLeftStdId, WheelLeftExtId, WheelDLC);
    //		rt_thread_delay(50);

    SendCanDataPage(&TxMessage, WheelData[2], WheelRTR, WheelIDE, WheelLeftStdId, WheelLeftExtId, WheelDLC);
    // rt_thread_delay(50);

    SendCanDataPage(&TxMessage, WheelData[3], WheelRTR, WheelIDE, WheelLeftStdId, WheelLeftExtId, WheelDLC);
    // rt_thread_delay(50);
    return 0;
  }
  // 右轮电机使能
  else if (num == 1) {
    SendCanDataPage(&TxMessage, WheelData[0], WheelRTR, WheelIDE, WheelRightStdId, WheelRightExtId, WheelDLC);
    // rt_thread_delay(50);

    SendCanDataPage(&TxMessage, WheelData[1], WheelRTR, WheelIDE, WheelRightStdId, WheelRightExtId, WheelDLC);
    // rt_thread_delay(50);

    SendCanDataPage(&TxMessage, WheelData[2], WheelRTR, WheelIDE, WheelRightStdId, WheelRightExtId, WheelDLC);
    // rt_thread_delay(50);

    SendCanDataPage(&TxMessage, WheelData[3], WheelRTR, WheelIDE, WheelRightStdId, WheelRightExtId, WheelDLC);
    // rt_thread_delay(50);
    return 0;
  } else {
    rt_kprintf("输入错误！\n");
    return -1;
  }
  //	rt_kprintf("填充的CAN1数据为：");
  //	CAN1_PrintSendData(&TxMessage);
  //	putchar(10);
}

// 设置默认速度
void SetDefaultSpeed(u8 num) {
  if (num == 2) {
    SendCanDataPage(&TxMessage, WheelData[5], WheelRTR, WheelIDE, WheelLeftStdId, WheelLeftExtId, WheelDLC);
    // rt_thread_delay(50);
  } else if (num == 1) {
    SendCanDataPage(&TxMessage, WheelData[6], WheelRTR, WheelIDE, WheelRightStdId, WheelRightExtId, WheelDLC);
    // rt_thread_delay(50);
  } else {
    rt_kprintf("输入错误！\n");
  }
}

// 单个电机停止
void SetWheelStop(u8 num) {
  if (num == 1) {
    SendCanDataPage(&TxMessage, WheelData[4], WheelRTR, WheelIDE, WheelRightStdId, WheelRightExtId, WheelDLC);
  } else if (num == 2) {
    SendCanDataPage(&TxMessage, WheelData[4], WheelRTR, WheelIDE, WheelLeftStdId, WheelLeftExtId, WheelDLC);
  } else {
    rt_kprintf("输入错误！\n");
  }
}

/*
功能：小车直行。
参数：
    speed：小车的速度
*/
void WheelSpeedSet(SBUS_CH_Struct* SBUS_CH) {
  int tempL = 0, tempR = 0;
  char dataL[8] = { 0 };
  char dataR[8] = { 0 };
  uint16_t speed = SBUS_CH->CH1;

  for (int i = 0; i < 8; i++) {
    dataL[i] = WheelData[2][i];
    dataR[i] = WheelData[2][i];
  }

  if (speed == 1024) {
    SetAllWheelStop();
  }

  else if (speed >= 353 && speed < 1024) {
    // rt_kprintf("%d\n",speed);
    tempL = 2083.1 - (2.036 * speed);
    tempR = 0xffffffff - (2083.1 - (2.036 * speed));

    dataL[4] = (tempL >> 24);
    dataL[5] = (tempL >> 16);
    dataL[6] = (tempL >> 8);
    dataL[7] = (tempL);

    dataR[4] = (tempR >> 24);
    dataR[5] = (tempR >> 16);
    dataR[6] = (tempR >> 8);
    dataR[7] = (tempR);
    //		for(int i = 0;i<8;i++)
    //		{
    //			rt_kprintf("%#x ",data[i]);
    //		}
    SendCanDataPage(&TxMessage, dataL, WheelRTR, WheelIDE, WheelLeftStdId, WheelLeftExtId, WheelDLC);
    SendCanDataPage(&TxMessage, dataR, WheelRTR, WheelIDE, WheelRightStdId, WheelRightExtId, WheelDLC);
    SendCanDataPage(&TxMessage, WheelData[3], WheelRTR, WheelIDE, WheelLeftStdId, WheelLeftExtId, WheelDLC);
    SendCanDataPage(&TxMessage, WheelData[3], WheelRTR, WheelIDE, WheelRightStdId, WheelRightExtId, WheelDLC);
    tempL = 0;
    tempR = 0;
  } else if (speed > 1024 && speed <= 1694) {
    // rt_kprintf("%d\n",speed);
    tempL = 0xffffffff - ((speed - 1024) * 2.037);
    tempR = (speed - 1024) * 2.037;

    dataL[4] = (tempL >> 24);
    dataL[5] = (tempL >> 16);
    dataL[6] = (tempL >> 8);
    dataL[7] = (tempL);

    dataR[4] = (tempR >> 24);
    dataR[5] = (tempR >> 16);
    dataR[6] = (tempR >> 8);
    dataR[7] = (tempR);
    //		for(int i = 0;i<8;i++)
    //		{
    //			rt_kprintf("%#x ",data[i]);
    //		}
    SendCanDataPage(&TxMessage, dataL, WheelRTR, WheelIDE, WheelLeftStdId, WheelLeftExtId, WheelDLC);
    SendCanDataPage(&TxMessage, dataR, WheelRTR, WheelIDE, WheelRightStdId, WheelRightExtId, WheelDLC);
    SendCanDataPage(&TxMessage, WheelData[3], WheelRTR, WheelIDE, WheelLeftStdId, WheelLeftExtId, WheelDLC);
    SendCanDataPage(&TxMessage, WheelData[3], WheelRTR, WheelIDE, WheelRightStdId, WheelRightExtId, WheelDLC);
    tempL = 0;
    tempR = 0;
  } else {
    rt_kprintf("输入电机速度数据错误！\n");
  }
}

// 双电机全部停转
void SetAllWheelStop(void) {
  SendCanDataPage(&TxMessage, WheelData[7], WheelRTR, WheelIDE, WheelAllStdId, WheelAllExtId, WheelDLC);
}

/*
功能：获取小车行走速度
参数：
    x：值为0至1之间的一个数，为x轴输入值占整个量程范围的占比
    y：同x，将x变为y
    speed：为存放x轴和y轴速度数据的地址
    driver_max_value：为电机可运行的最大速度
*/
void coordinate_transformation(double x, double y, int* speed, int driver_max_value) // 坐标转换
{
  static double Cx = 0.5;
  static double Cy = 1;
  double right_rpm = 0;
  double left_rpm = 0;

  left_rpm = -Cx * (x * driver_max_value) + Cy * (y * driver_max_value);
  speed[0] = left_rpm;

  right_rpm = -Cx * (x * driver_max_value) - Cy * (y * driver_max_value);
  speed[1] = right_rpm;

  //    left_rpm = Cx *(x* driver_max_value)+ Cy*(y*driver_max_value);
  //                speed[0] = left_rpm;
  //
  //    right_rpm = -Cx *(x* driver_max_value)+ Cy*(y*driver_max_value);
  //                speed[1] = right_rpm;
}

/*
功能：控制小车运行
参数：
    speed：指向左右轮速度数据地址的指针

*/
void TowMotorCarRun(int* speed) {

  char dataL[8] = { 0 };
  char dataR[8] = { 0 };
  for (int i = 0; i < 8; i++) {
    dataL[i] = WheelData[2][i];
    dataR[i] = WheelData[2][i];
  }

  dataL[4] = (speed[0] >> 24);
  dataL[5] = (speed[0] >> 16);
  dataL[6] = (speed[0] >> 8);
  dataL[7] = (speed[0]);

  dataR[4] = (speed[1] >> 24);
  dataR[5] = (speed[1] >> 16);
  dataR[6] = (speed[1] >> 8);
  dataR[7] = (speed[1]);

  // rt_kprintf("dataR[6] = %#x dataR[7] = %#x\n",dataR[6],dataR[7]);
  if (speed[0] > 506)
    SendCanDataPage(&TxMessage, WheelData[5], WheelRTR, WheelIDE, WheelLeftStdId, WheelLeftExtId, WheelDLC);
  else if (speed[0] < -506)
    SendCanDataPage(&TxMessage, WheelData[6], WheelRTR, WheelIDE, WheelLeftStdId, WheelLeftExtId, WheelDLC);
  else
    SendCanDataPage(&TxMessage, dataL, WheelRTR, WheelIDE, WheelLeftStdId, WheelLeftExtId, WheelDLC);

  if (speed[1] > 506)
    SendCanDataPage(&TxMessage, WheelData[5], WheelRTR, WheelIDE, WheelRightStdId, WheelRightExtId, WheelDLC);
  else if (speed[1] < -506)
    SendCanDataPage(&TxMessage, WheelData[6], WheelRTR, WheelIDE, WheelRightStdId, WheelRightExtId, WheelDLC);
  else
    SendCanDataPage(&TxMessage, dataR, WheelRTR, WheelIDE, WheelRightStdId, WheelRightExtId, WheelDLC);

  SendCanDataPage(&TxMessage, WheelData[3], WheelRTR, WheelIDE, WheelLeftStdId, WheelLeftExtId, WheelDLC);
  SendCanDataPage(&TxMessage, WheelData[3], WheelRTR, WheelIDE, WheelRightStdId, WheelRightExtId, WheelDLC);
}

/*
功能：电机控制直接调用的函数，由使能函数和速度设置函数组成
参数：

*/
void WheelEnableAll(void) {
  WheelEnable(1);
  WheelEnable(2);
}

/*---------------------------------------------------------------------四轮驱动相关函数------------------------------------------------------------------*/

/*
功能：四电机控制小车行走
参数：
    speed：speed[0]要用在左侧轮子 和 speed[1]要用在右侧轮子  的速度
*/

extern SBUS_CH_Struct SBUS_CH;

static int speed_0 = 0;
static int speed_1 = 0;
void FourMotorRun(int* speed) {
  char dataL1[8] = { 0 };
  char dataL2[8] = { 0 };
  char dataR1[8] = { 0 };
  char dataR2[8] = { 0 };
  for (int i = 0; i < 8; i++) {
    dataL1[i] = WheelData[0][i];
    //dataR1[i] = WheelData[14][i];
    //dataL2[i] = WheelData[15][i];
    //dataR2[i] = WheelData[14][i];
  }
  // rt_kprintf("%d \n",SBUS_CH.signal[23]);
  rt_kprintf("speed[0]: %d \n", speed[0]);
  rt_kprintf("speed[1]: %d \n", speed[1]);
  speed_0 = (int)speed[0];
  speed_1 = (int)speed[1];

	

  if (SBUS_CH.signal[23] == SBUS_CONNECT_FLAG) {
    //		Delay_Ms(200);
    //		rt_kprintf("连接上了\n");
    if (ReadIO_IN_Val(1) == 1) // 如果未检测到障碍物
    {
			dataL1[1] = (speed[0]);
			dataL1[2] = (speed[0]);
      dataL1[3] = (speed[0]);
      dataL1[4] = (speed[0] >> 8);
      dataL1[5] = 0;
      dataL1[6] = 0;
/*
      dataL2[4] = (-speed[0]);
      dataL2[5] = ((-speed[0]) >> 8);
      dataL2[6] = 0;
      dataL2[7] = 0;

      dataR1[4] = (-speed[1]);
      dataR1[5] = ((-speed[1]) >> 8);
      dataR1[6] = 0;
      dataR1[7] = 0;

      dataR2[4] = (speed[1]);
      dataR2[5] = (speed[1] >> 8);
      dataR2[6] = 0;
      dataR2[7] = 0;
*/
			SendCanDataPage(&TxMessage, dataL1, WheelRTR, Motor1IDE, 0xfff, 0x18ff2100, WheelDLC); //
			
			
			
      // rt_kprintf("dataR[6] = %#x dataR[7] = %#x\n",dataR[6],dataR[7]);
      if (speed[0] > 506) // 限制左轮最大速度
      {
        SendCanDataPage(&TxMessage, WheelData[9], WheelRTR, WheelIDE, 0x601, WheelLeftExtId,
                        WheelDLC); // 左前轮正转506
        SendCanDataPage(&TxMessage, WheelData[11], WheelRTR, WheelIDE, 0x602, WheelLeftExtId,
                        WheelDLC); // 左后轮正转506  传入负值
      } else if (speed[0] < -506)  // 限制左轮最大速度
      {
        SendCanDataPage(&TxMessage, WheelData[11], WheelRTR, WheelIDE, 0x601, WheelLeftExtId,
                        WheelDLC); // 左前轮倒转506
        SendCanDataPage(&TxMessage, WheelData[9], WheelRTR, WheelIDE, 0x602, WheelLeftExtId,
                        WheelDLC); // 左后轮倒转506  传入正值
      } else {
// modify address ID 20260203
        SendCanDataPage(&TxMessage, dataL1, WheelRTR, WheelIDE, 0xfff, WheelLeftExtId, WheelDLC); //
        SendCanDataPage(&TxMessage, dataL2, WheelRTR, WheelIDE, 0xfff, WheelLeftExtId, WheelDLC);
      }

      if (speed[1] > 506) // 限制右轮最大速度
      {
        SendCanDataPage(&TxMessage, WheelData[8], WheelRTR, WheelIDE, 0x601, WheelRightExtId,
                        WheelDLC); // 右前轮正转506
        SendCanDataPage(&TxMessage, WheelData[10], WheelRTR, WheelIDE, 0x602, WheelRightExtId,
                        WheelDLC); // 右后轮正转506  传入负值
      } else if (speed[1] < -506)  // 限制右轮最大速度
      {
        SendCanDataPage(&TxMessage, WheelData[10], WheelRTR, WheelIDE, 0x601, WheelRightExtId,
                        WheelDLC); // 右前轮倒转506
        SendCanDataPage(&TxMessage, WheelData[8], WheelRTR, WheelIDE, 0x602, WheelRightExtId,
                        WheelDLC); // 右后轮倒转506  传入正值
      } else                       // 正常给速度
      {
        SendCanDataPage(&TxMessage, dataR1, WheelRTR, WheelIDE, 0x601, WheelRightExtId, WheelDLC);
        SendCanDataPage(&TxMessage, dataR2, WheelRTR, WheelIDE, 0x602, WheelRightExtId, WheelDLC);
      }
    } else if (ReadIO_IN_Val(1) == 0) // 检测到有障碍物
    {

      if (((speed[0] == (-speed[1])) && (speed[0] < 0) && (speed[1] > 0)) ==
          0) // 如果小车不进行向后行驶，小车失能，也就是小车只能进行向后行驶
      {
        SendCanDataPage(&TxMessage, WheelData[15], WheelRTR, WheelIDE, 0x601, WheelLeftExtId,
                        WheelDLC); // 左前轮0
        SendCanDataPage(&TxMessage, WheelData[14], WheelRTR, WheelIDE, 0x601, WheelLeftExtId,
                        WheelDLC); // 右前轮0
        SendCanDataPage(&TxMessage, WheelData[15], WheelRTR, WheelIDE, 0x602, WheelLeftExtId,
                        WheelDLC); // 左后轮0
        SendCanDataPage(&TxMessage, WheelData[14], WheelRTR, WheelIDE, 0x602, WheelLeftExtId,
                        WheelDLC); // 左后轮0
      } else                       // 如果小车后退则正常行走
      {
        dataL1[4] = (speed[0]);
        dataL1[5] = (speed[0] >> 8);
        dataL1[6] = 0;
        dataL1[7] = 0;

        dataL2[4] = (-speed[0]);
        dataL2[5] = ((-speed[0]) >> 8);
        dataL2[6] = 0;
        dataL2[7] = 0;

        dataR1[4] = (-speed[1]);
        dataR1[5] = ((-speed[1]) >> 8);
        dataR1[6] = 0;
        dataR1[7] = 0;

        dataR2[4] = (speed[1]);
        dataR2[5] = (speed[1] >> 8);
        dataR2[6] = 0;
        dataR2[7] = 0;

        // rt_kprintf("dataR[6] = %#x dataR[7] = %#x\n",dataR[6],dataR[7]);
        if (speed[0] > 506) {
          SendCanDataPage(&TxMessage, WheelData[9], WheelRTR, WheelIDE, 0x601, WheelLeftExtId,
                          WheelDLC); // 左前轮正转506
          SendCanDataPage(&TxMessage, WheelData[11], WheelRTR, WheelIDE, 0x602, WheelLeftExtId,
                          WheelDLC); // 左后轮正转506  传入负值
        } else if (speed[0] < -506) {
          SendCanDataPage(&TxMessage, WheelData[11], WheelRTR, WheelIDE, 0x601, WheelLeftExtId,
                          WheelDLC); // 左前轮倒转506
          SendCanDataPage(&TxMessage, WheelData[9], WheelRTR, WheelIDE, 0x602, WheelLeftExtId,
                          WheelDLC); // 左后轮倒转506  传入正值
        } else {

          SendCanDataPage(&TxMessage, dataL1, WheelRTR, WheelIDE, 0x601, WheelLeftExtId, WheelDLC); //
          SendCanDataPage(&TxMessage, dataL2, WheelRTR, WheelIDE, 0x602, WheelLeftExtId, WheelDLC);
        }

        if (speed[1] > 506) {
          SendCanDataPage(&TxMessage, WheelData[8], WheelRTR, WheelIDE, 0x601, WheelRightExtId,
                          WheelDLC); // 右前轮正转506
          SendCanDataPage(&TxMessage, WheelData[10], WheelRTR, WheelIDE, 0x602, WheelRightExtId,
                          WheelDLC); // 右后轮正转506  传入负值
        } else if (speed[1] < -506) {
          SendCanDataPage(&TxMessage, WheelData[10], WheelRTR, WheelIDE, 0x601, WheelRightExtId,
                          WheelDLC); // 右前轮倒转506
          SendCanDataPage(&TxMessage, WheelData[8], WheelRTR, WheelIDE, 0x602, WheelRightExtId,
                          WheelDLC); // 右后轮倒转506  传入正值
        } else {
          SendCanDataPage(&TxMessage, dataR1, WheelRTR, WheelIDE, 0x601, WheelRightExtId, WheelDLC);
          SendCanDataPage(&TxMessage, dataR2, WheelRTR, WheelIDE, 0x602, WheelRightExtId, WheelDLC);
        }
      }
    }
  } else // 若sbus遥控器未连接上，则小车停止
  {
    //		Delay_Ms(1000);
    //		rt_kprintf("未连接上\n");
    SendCanDataPage(&TxMessage, WheelData[15], WheelRTR, WheelIDE, 0x601, WheelLeftExtId,
                    WheelDLC); // 左前轮0
    SendCanDataPage(&TxMessage, WheelData[14], WheelRTR, WheelIDE, 0x601, WheelLeftExtId,
                    WheelDLC); // 右前轮0
    SendCanDataPage(&TxMessage, WheelData[15], WheelRTR, WheelIDE, 0x602, WheelLeftExtId,
                    WheelDLC); // 左后轮0
    SendCanDataPage(&TxMessage, WheelData[14], WheelRTR, WheelIDE, 0x602, WheelLeftExtId,
                    WheelDLC); // 右后轮0
  }
}

/*
功能：开启线程通过sbus和can控制小车行走
参数：/

*/


static void Sbus_thread_entry(void* parameter) {
  double x = 0;
  double y = 0;
  u8 num = *(u8*)parameter;

  while (1) {

		
    x = (1024 - SBUS_CH.CH2) / 670.5;              /*输入x轴的量程占比*/
    y = (1024 - SBUS_CH.CH1) / 670.5;              /*输入y轴的量程占比*/

    coordinate_transformation(x, y, LRSpeed, 506); /*将速度值计算好取出*/
		rt_kprintf("xxxxx%d %d\n", LRSpeed[0], LRSpeed[1]);
    FourMotorRun(LRSpeed); 
		
		
		if (num == 2) {
      // WheelEnableAll();
      // rt_kprintf("22222%d %d\n",LRSpeed[0],LRSpeed[1]);
      TowMotorCarRun(LRSpeed); 
    } else if (num == 4) {
      // rt_kprintf("44444%d %d\n",LRSpeed[0],LRSpeed[1]);
      FourMotorRun(LRSpeed); 
    } else {
      rt_kprintf("输入错误！\n");
    }

		
    //		WheelSpeedSet(&SBUS_CH);
    rt_thread_delay(1); // 在main函数运行开启线程的时候需要线程让出CPU资源，保证cpu可以运行开启后面的线程。
  }
}

/* 定义线程控制块 */
static rt_thread_t Sbus_thread = RT_NULL;

int bsp_Sbus_thread(u8 choose) {

  Sbus_thread =                           /* 线程控制块指针 */
      rt_thread_create("Sbus",            /* 线程名字 */
                       Sbus_thread_entry, /* 线程入口函数 */
                       &choose,           /* 线程入口函数参数 */
                       512,               /* 线程栈大小 */
                       3,                 /* 线程的优先级 */
                       20);               /* 线程时间片 */

  /* 启动线程，开启调度 */
  if (Sbus_thread != RT_NULL)
    rt_thread_startup(Sbus_thread);
  else
    return -1;
}
