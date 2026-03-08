/**
  ******************************************************************************
  * @file    WIZnet MDK5 Project Template 
  * @author  WIZnet Software Team
  * @version V1.0.0
  * @date    2018-09-25
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2018 WIZnet H.K. Ltd.</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/

#include "232_2.h"
#include "wizchip_conf.h"
#include "spi.h"
#include "socket.h"
#include "delay.h"
#include "main.h"
/**
  * @brief  配置指定SPI的引脚
  * @retval None
  */
static void SPI_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	#if 1
	//打开时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	//打开端口复用
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	//PB13->SCK	 					 
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;  	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	//PB14->MISO
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_14;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	//PB15->MOSI	
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	//PB12->CS,初始化片选输出引脚
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	//拉高片选
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	
	#else 
	//打开时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB,ENABLE);
	
	//打开端口复用
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);
	
	//PB3->SCK	 					 
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;  	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	//PB4->MISO
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_4;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	//PB5->MOSI	
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	//PA4->CS,初始化片选输出引脚
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	//拉高片选
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	#endif
}
/**
  * @brief  根据外部SPI设备配置SPI相关参数
  * @retval None
  */
void SPI_Configuration(void)
{
	SPI_InitTypeDef SPI_InitStruct;

	SPI_GPIO_Configuration();
	#if 1	
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStruct.SPI_Direction= SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2,&SPI_InitStruct);
	
	SPI_Cmd(SPI2, ENABLE);
	#else
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStruct.SPI_Direction= SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1,&SPI_InitStruct);
	
	SPI_Cmd(SPI1, ENABLE);
	#endif
}
/**
  * @brief  写1字节数据到SPI总线
  * @param  TxData 写到总线的数据
  * @retval None
  */
void SPI_WriteByte(uint8_t TxData)
{	
	#if 1
	while((SPI2->SR&SPI_I2S_FLAG_TXE)==(uint16_t)RESET);	//等待发送区空		  
	SPI2->DR=TxData;	 	  									              //发送一个byte 
	while((SPI2->SR&SPI_I2S_FLAG_RXNE)==(uint16_t)RESET); //等待接收完一个byte  
	SPI2->DR;	
	#else
	while((SPI1->SR&SPI_I2S_FLAG_TXE)==(uint16_t)RESET);	//等待发送区空		  
	SPI1->DR=TxData;	 	  									//发送一个byte 
	while((SPI1->SR&SPI_I2S_FLAG_RXNE)==(uint16_t)RESET); //等待接收完一个byte  
	SPI1->DR;	
	#endif	
}
/**
  * @brief  从SPI总线读取1字节数据
  * @retval 读到的数据
  */
uint8_t SPI_ReadByte(void)
{	
	#if 1
	while((SPI2->SR&SPI_I2S_FLAG_TXE)==(uint16_t)RESET);	//等待发送区空			  
	SPI2->DR=0xFF;	 	  										//发送一个空数据产生输入数据的时钟 
	while((SPI2->SR&SPI_I2S_FLAG_RXNE)==(uint16_t)RESET); //等待接收完一个byte  
	return SPI2->DR;	
  #else 
	while((SPI1->SR&SPI_I2S_FLAG_TXE)==(uint16_t)RESET);	//等待发送区空			  
	SPI1->DR=0xFF;	 	  										//发送一个空数据产生输入数据的时钟 
	while((SPI1->SR&SPI_I2S_FLAG_RXNE)==(uint16_t)RESET); //等待接收完一个byte  	
	return SPI1->DR;	
  #endif	
}
/**
  * @brief  进入临界区
  * @retval None
  */
void SPI_CrisEnter(void)
{
	__set_PRIMASK(1);
}
/**
  * @brief  退出临界区
  * @retval None
  */
void SPI_CrisExit(void)
{
	__set_PRIMASK(0);
}

/**
  * @brief  片选信号输出低电平
  * @retval None
  */
void SPI_CS_Select(void)
{
	#if 1
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	#else 
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	#endif
}
/**
  * @brief  片选信号输出高电平
  * @retval None
  */
void SPI_CS_Deselect(void)
{
	#if 1
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	#else 
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	#endif
}
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SOCK_TCPS        0
#define DATA_BUF_SIZE   2048
/* Private macro -------------------------------------------------------------*/
uint8_t gDATABUF[DATA_BUF_SIZE];
// 默认网络配置
wiz_NetInfo gWIZNETINFO = { .mac = {0x00, 0x08, 0xdc,0x11, 0x11, 0x11},
                            .ip = {10, 0, 0, 160},
                            .sn = {255,255,255,0},
                            .gw = {192, 168, 1, 1},
                            .dns = {8,8,8,8},
                            .dhcp = NETINFO_STATIC };

void network_init(void)
{

	uint8_t tmp;
	uint8_t memsize[2][8] = {{2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};
	// 首先，应该注册用户为访问WIZCHIP实现的SPI回调函数 
	/* 临界区回调 */
	reg_wizchip_cris_cbfunc(SPI_CrisEnter, SPI_CrisExit);	//注册临界区函数
	/* 片选回调 */
#if   _WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_VDM_
	reg_wizchip_cs_cbfunc(SPI_CS_Select, SPI_CS_Deselect);  //注册SPI片选信号函数
#elif _WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_FDM_
	reg_wizchip_cs_cbfunc(SPI_CS_Select, SPI_CS_Deselect);  // CS must be tried with LOW.
#else
   #if (_WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_SIP_) != _WIZCHIP_IO_MODE_SIP_
      #error "Unknown _WIZCHIP_IO_MODE_"
   #else
      reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
   #endif
#endif
	/* SPI读和写回调函数 */
	reg_wizchip_spi_cbfunc(SPI_ReadByte, SPI_WriteByte);	//注册读写函数
	
	/* WIZCHIP SOCKET Buffer 初始化 */
	if(ctlwizchip(CW_INIT_WIZCHIP,(void*)memsize) == -1){
		 rt_kprintf("WIZCHIP Initialized fail.\r\n");
		 while(1);
	}
	rt_kprintf("WIZCHIP Initialized success.\r\n");
	/* PHY链路状态检查 */
	do{
		 if(ctlwizchip(CW_GET_PHYLINK, (void*)&tmp) == -1){
				rt_kprintf("Unknown PHY Link stauts.\r\n");
		 }
	}while(tmp == PHY_LINK_OFF);
	
	uint8_t tmpstr[6];
	ctlnetwork(CN_SET_NETINFO, (void*)&gWIZNETINFO);
	ctlnetwork(CN_GET_NETINFO, (void*)&gWIZNETINFO);

	// Display Network Information

	ctlwizchip(CW_GET_ID,(void*)tmpstr);
	rt_kprintf("\r\n=== %s NET CONF ===\r\n",(char*)tmpstr);
	rt_kprintf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",gWIZNETINFO.mac[0],gWIZNETINFO.mac[1],gWIZNETINFO.mac[2],
		  gWIZNETINFO.mac[3],gWIZNETINFO.mac[4],gWIZNETINFO.mac[5]);
	rt_kprintf("SIP: %d.%d.%d.%d\r\n", gWIZNETINFO.ip[0],gWIZNETINFO.ip[1],gWIZNETINFO.ip[2],gWIZNETINFO.ip[3]);
	rt_kprintf("GAR: %d.%d.%d.%d\r\n", gWIZNETINFO.gw[0],gWIZNETINFO.gw[1],gWIZNETINFO.gw[2],gWIZNETINFO.gw[3]);
	rt_kprintf("SUB: %d.%d.%d.%d\r\n", gWIZNETINFO.sn[0],gWIZNETINFO.sn[1],gWIZNETINFO.sn[2],gWIZNETINFO.sn[3]);
	rt_kprintf("DNS: %d.%d.%d.%d\r\n", gWIZNETINFO.dns[0],gWIZNETINFO.dns[1],gWIZNETINFO.dns[2],gWIZNETINFO.dns[3]);
	rt_kprintf("======================\r\n");
}

/**
  * @brief  Loopback Test Example Code using ioLibrary_BSD	
  * @retval None
  */
void platform_init(void)
{
	SPI_Configuration();
}



/*写代码时测试用的函数*/
void w5500_test()
{
	
	uint16_t len=0;
	uint8_t DstIP[4]={10,0,0,162};    //目的IP地址
	uint16_t	DstPort = 8080;           //端口号
	network_init();
	while(1)
	{
		switch(getSn_SR(SOCK_TCPS))														// 获取socket0的状态
		{
			case SOCK_INIT:															// Socket处于初始化完成(打开)状态	
					connect(SOCK_TCPS,DstIP,DstPort);
			break;
			case SOCK_ESTABLISHED:											// Socket处于连接建立状态
					if(getSn_IR(SOCK_TCPS) & Sn_IR_CON)   					
					{
						setSn_IR(SOCK_TCPS, Sn_IR_CON);								// Sn_IR的CON位置1，通知W5500连接已建立
					}
					// 数据回环测试程序：数据从上位机服务器发给W5500，W5500接收到数据后再回给服务器
					len=getSn_RX_RSR(SOCK_TCPS);										// len=Socket0接收缓存中已接收和保存的数据大小					
					if(len)
					{
						recv(SOCK_TCPS,gDATABUF,len);		
						rt_kprintf("%s\r\n",gDATABUF);
						send(SOCK_TCPS,gDATABUF,len);							
					}											
			break;
			case SOCK_CLOSE_WAIT:												  // Socket处于等待关闭状态
				disconnect(SOCK_TCPS);	
			break;
			case SOCK_CLOSED:														// Socket处于关闭状态
					socket(SOCK_TCPS,Sn_MR_TCP,5000,0x00);		// 打开Socket0，打开一个本地端口
			break;
	   }
		rt_kprintf("hello world");
   }
}


/*
网口功能的线程函数
功能：实现网口通信，这部分写的是客户端client
*/
static void w5500_thread_entry(void *parameter)
{
	/* Network initialization */
	
	network_init();
	
	uint16_t len=0;
	uint8_t DstIP[4]={10,0,0,162};
	uint16_t DstPort=8080;
	
	while(1)
	{	
		switch(getSn_SR(SOCK_TCPS))														// 获取socket0的状态
		{
			case SOCK_INIT:																// Socket处于初始化完成(打开)状态	
					connect(SOCK_TCPS,DstIP,DstPort);
			break;
			case SOCK_ESTABLISHED:														// Socket处于连接建立状态
					if(getSn_IR(SOCK_TCPS) & Sn_IR_CON)   						
					{
						setSn_IR(SOCK_TCPS, Sn_IR_CON);									// Sn_IR的CON位置1，通知W5500连接已建立
					}
					// 数据回环测试程序：数据从上位机服务器发给W5500，W5500接收到数据后再回给服务器
					len=getSn_RX_RSR(SOCK_TCPS);										// len=Socket0接收缓存中已接收和保存的数据大小					
					if(len)
					{
						recv(SOCK_TCPS,gDATABUF,len);		
						rt_kprintf("%s\r\n",gDATABUF);
						send(SOCK_TCPS,gDATABUF,len);							
					}											
			break;
			case SOCK_CLOSE_WAIT:												  // Socket处于等待关闭状态
				disconnect(SOCK_TCPS);	
			break;
			case SOCK_CLOSED:														// Socket处于关闭状态
					socket(SOCK_TCPS,Sn_MR_TCP,5000,0x00);		// 打开Socket0，打开一个本地端口
			break;
	    }
		
	}	
}


/* 定义网口的线程控制块 */
static rt_thread_t w5500_thread = RT_NULL;
int bsp_w5500_thread(void)
{
	w5500_thread =                          /* 线程控制块指针 */
    rt_thread_create( "w5500",              /* 线程名字 */
                      w5500_thread_entry,   /* 线程入口函数 */
                      RT_NULL,             /* 线程入口函数参数 */
                      512,                 /* 线程栈大小 */
                      3,                   /* 线程的优先级 */
                      20);                 /* 线程时间片 */
                   
    /* 启动线程，开启调度 */
   if (w5500_thread != RT_NULL)
        rt_thread_startup(w5500_thread);
    else
        return -1;   
}
/*********************************END OF FILE**********************************/

