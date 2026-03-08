
#include "bsp_sd.h"	
/*
函数功能：SD卡底层接口,通过SPI时序向SD卡读写一个字节
函数参数：data是要写入的数据
返 回 值：读到的数据
*/
//u8 SDCardReadWriteOneByte(u8 DataTx)
//{		 
//	
//    u8 i;
//    u8 data=0;
//    for(i=0;i<8;i++)
//    {
//        SDCARD_SCK(0);
//        if(DataTx&0x80)SDCARD_MOSI(1);
//        else SDCARD_MOSI(0);
//        SDCARD_SCK(1);
//        DataTx<<=1;
//        
//        data<<=1;
//			//SDCARD_MISO(1);
//        if(SDCARD_MISOin)data|=0x01;
//    }
//    return data;
//}


//4种: 边沿两种、电平是两种
/*
函数功能：底层SD卡接口初始化

本程序SPI接口如下：
PC11  片选 SDCardCS
PC12  时钟 SDCardSCLK
PD2   输出 SPI_MOSI--主机输出从机输入
PC8   输入 SPI_MISO--主机输入从机输出
*/
//void SDCardSpiInit(void)
//{
//  /*1. 开启时钟*/
// 	RCC->APB2ENR|=1<<5;		    //使能PORTD时钟
//	RCC->APB2ENR|=1<<4;		    //使能PORTC时钟
//  
//  /*2. 配置GPIO口模式*/
//  GPIOC->CRH&=0xFFF00FF0;
//  GPIOC->CRH|=0x00033008;
//  
//  GPIOD->CRL&=0xFFFFF0FF;
//  GPIOD->CRL|=0x00000300;
//  
//  /*3. 上拉*/
//  GPIOC->ODR|=1<<8;
//  GPIOC->ODR|=1<<11;
//  GPIOC->ODR|=1<<12;
//  GPIOD->ODR|=1<<2;
//}

/*
函数功能：从sd卡读取一个数据包的内容
函数参数：
				buf:数据缓存区
				len:要读取的数据长度.
返回值：
			0,成功;其他,失败;	
*/
u8 SDCardRecvData(u8*buf,u16 len)
{			  	  
	while(MySPI_SwapByte(0xFF)!=0xFE){}//等待SD卡发回数据起始令牌0xFE 
    while(len--)//开始接收数据
    {
        *buf=MySPI_SwapByte(0xFF);
        buf++;
    }
    //下面是2个伪CRC（dummy CRC）
    MySPI_SwapByte(0xFF);
    MySPI_SwapByte(0xFF);									  					    
    return 0;//读取成功
}


/*
函数功能：向sd卡写入一个数据包的内容 512字节
函数参数：
					buf 数据缓存区
					cmd 指令
返 回 值：0表示成功;其他值表示失败;
*/
u8 SDCardSendData(u8*buf,u8 cmd)
{	
	u16 t;		  	  
	while(MySPI_SwapByte(0xFF)!=0xFF){}  //等待忙状态
	MySPI_SwapByte(cmd);
	if(cmd!=0xFD)//不是结束指令
	{
		  for(t=0;t<512;t++)MySPI_SwapByte(buf[t]);//提高速度,减少函数传参时间
	    MySPI_SwapByte(0xFF); //忽略crc
	    MySPI_SwapByte(0xFF);
		  MySPI_SwapByte(0xFF); //接收响应								  					    
	}						 									  					    
  return 0;//写入成功
}


/*
函数功能：向SD卡发送一个命令
函数参数：
				u8 cmd   命令 
				u32 arg  命令参数
				u8 crc   crc校验值	
返回值:SD卡返回的响应
*/											

/*对SPI协议的理解：原协议中从前到后，1位的起始位固定为0 1位的传输标志位为1（用1代表传输的是个命令，0代表传输的响应），
6位的cmd命令码这样加起来就是8位，在SPI中一次要发送8个字节，所以前八位为，起始位加传输标志位，加命令码，一般我们用
cmd|0x40因为前两位01构成0x40，然后或上后面6位命令码。中间为32位的地址位或参数位（读写命令为地址，因为需要寻址，其他命
令为参数），高位先行，所以要分四批每次一个字节发送数据，发完这些，在后面是7位的CRC校验码加1位的停止位，停止位固定为1
合起来一个字节一起发送*/

u8 SendSDCardCmd(u8 cmd, u32 arg, u8 crc)
{
	u8 r1;	
	MySPI_Stop(); //取消上次片选
 	MySPI_SwapByte(0xff);//提供额外的8个时钟
	MySPI_Start(); //选中SD卡
	while(MySPI_SwapByte(0xFF)!=0xFF)
	{
		printf("MySPI_SwapByte(0xFF)=%#X\r\n",MySPI_SwapByte(0xFF));
	};//等待成功

	//发送数据
	MySPI_SwapByte(cmd | 0x40);//分别写入命令
	MySPI_SwapByte(arg >> 24);
	MySPI_SwapByte(arg >> 16);
	MySPI_SwapByte(arg >> 8);
	MySPI_SwapByte(arg);	  
	MySPI_SwapByte(crc); 
	
	if(cmd==SDCard_CMD12)
		MySPI_SwapByte(0xff);//Skip a stuff byte when stop reading
	do
	{
		r1=MySPI_SwapByte(0xFF);
	}while(r1&0x80);	  //等待响应，或超时退出
	return r1;
}


/*
函数功能：获取SD卡的总扇区数（扇区数）   
返 回 值：
				0表示容量检测出错，其他值表示SD卡的容量(扇区数/512字节)
说   明：
				每扇区的字节数必为512字节，如果不是512字节，则初始化不能通过.	
*/
u32 GetSDCardSectorCount(void)
{
	u8 i=0;
    u8 csd[16];
    u32 Capacity=0;  
	u16 csize;
	//获取SD卡的CSD信息，包括容量和速度信息,存放CID的内存,至少16Byte
	SendSDCardCmd(SDCard_CMD9,0,0x01);//发SDCard_CMD9命令，读CSD
	printf("11");
	SDCardRecvData(csd,16);//接收16个字节的数据 
	MySPI_Stop();//取消片选
	MySPI_SwapByte(0xff);//提供额外的8个时钟

    if((csd[0]&0xC0)==0x40)  //SDHC卡，按照下面方式计算
    {	
		printf("22");
		csize=csd[9]+(csd[8]<<8)+1;
		Capacity=csize<<10;//得到扇区数	 	
		for(i=0;i<16;i++)
		{
			printf("csd[%d]=%d",i,csd[i]);
		}
    }
	printf("33");
    return Capacity;
}


/*
函数功能：  初始化SD卡
返 回 值：  非0表示初始化失败!
步	  骤：  软件SPI初始化操作，时钟线先发74个脉冲信号，然后在片选上该卡的同时发送复位命令
		    CMD0，在SPI协议中第一次发送复位命令时要发送CRC校验码，CMD0的校验码为0x95，成功后后面的数据
		    便不需要再发送校验码。然后需要鉴别是否是2.0版本，发送CMD8命令，此时便不需要发
			校验码但校验码最低位必须为1因为数据包结束标志必须为1.若识别成功是2.0版本，则
			需要发送ACMD41命令，发送HCS参数置1，通过接收响应的HCS位判别SD卡是支持高容量
			还是不支持，1为支持，0为不支持，若支持则进行下一步发送CMD58命令，看看到底支
			持什么容量的SD卡，响应会返回OCR寄存器的值，在该寄存器第30位上表示卡的容量状态
			，为1的话表示该卡位SDHC卡大于2G，若为0则表示SDSC卡，表示小于2G。
*/
u8 SDCardDeviceInit(void)
{
  u8 r1=0;      // 存放SD卡的返回值
  u8 data;  
	u16 i;

	/*2. 发送最少74个脉冲*/
	
 	for(i=0;i<10;i++)
	MySPI_SwapByte(0xFF);
	
	//片选使能
	MySPI_Start();
	/*3. 进入闲置状态*/
	do
	{
		printf("r1=%d\r\n",r1);
		r1=SendSDCardCmd(SDCard_CMD0,0,0x95);										//会返回1
	}while(r1!=0x01);																//复位成功后会返回1
	printf("成功！\n");
	/*4.鉴别是否是2.0版本的SD卡*/
	if(SendSDCardCmd(SDCard_CMD8,0x1AA,0x87)==0x01)
	{
		do
		{
			SendSDCardCmd(SDCard_CMD55,0,0x01);	    								//发送SDCard_CMD55
			r1=SendSDCardCmd(SDCard_CMD41,0x40000000,0x01);							//发送SDCard_CMD41  与上一条命令联合起来使用测试是SDSC还是SDHC卡
		}while(r1);
		
		if(SendSDCardCmd(SDCard_CMD58,0,0x01) == 0)//鉴别SD2.0卡版本开始
		{
			data=MySPI_SwapByte(0xFF);//得到OCR值
			if(data&0x40)
			{
				r1=0; //高速卡
			}					
		}
	} 
	MySPI_Stop();//取消片选
 	MySPI_SwapByte(0xff);//提供额外的8个时钟
	return r1;          //其他错误
}


/*
函数功能：读SD卡
函数参数：
				buf:数据缓存区
				sector:扇区
				cnt:扇区数
返回值:
				0,ok;其他,失败.
说  明：
				SD卡一个扇区大小512字节
*/
void SDCardReadData(u8*buf,u32 sector,u32 cnt)
{
	u32 i=0;
	if(cnt==1)
	{
		SendSDCardCmd(SDCard_CMD17,sector,0x01);//读扇区
		SDCardRecvData(buf,512);			//接收512个字节	   
	}
	else
	{
		SendSDCardCmd(SDCard_CMD18,sector,0x01);//连续读命令
		for(i=0;i<cnt;i++)
		{
			SDCardRecvData(buf,512);//接收512个字节	 
			buf+=512;  
		}
		SendSDCardCmd(SDCard_CMD12,0,0x01);	//停止数据传输
	}   
	MySPI_Stop();//取消片选
 	MySPI_SwapByte(0xff);//提供额外的8个时钟();
}


/*
函数功能：向SD卡写数据
函数参数：
				buf:数据缓存区
				sector:起始扇区
				cnt:扇区数
说  明：
				SD卡一个扇区大小512字节
*/
void SDCardWriteData(u8*buf,u32 sector,u32 cnt)
{
	u32 i=0;
	if(cnt==1)
	{
		SendSDCardCmd(SDCard_CMD24,sector,0x01);//写单个扇区
		SDCardSendData(buf,0xFE);//写512个字节	   
	}
	else
	{
		SendSDCardCmd(SDCard_CMD55,0,0x01);	
		SendSDCardCmd(SDCard_CMD23,cnt,0x01);   //设置多扇区写入前预先擦除N个block
 		SendSDCardCmd(SDCard_CMD25,sector,0x01);//写多个扇区
		for(i=0;i<cnt;i++)
		{
			SDCardSendData(buf,0xFC);//写512个字节	 
			buf+=512;  
		}
		SDCardSendData(0,0xFD);//写结束指令
	}
	MySPI_Stop();
 	MySPI_SwapByte(0xff);//提供额外的8个时钟
}
