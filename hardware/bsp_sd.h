#ifndef BSP_SD_H_
#define BSP_SD_H_	 
#include "led.h"
#include "232_2.h"
#include "MySPI.h"
/*----------------------------------------------
本程序SPI接口如下：
PD7  片选 SDCardCS
PC13  时钟 SDCardSCLK
PB4   输出 SPI_MOSI--主机输出从机输入
PD4   输入 SPI_MISO--主机输入从机输出
------------------------------------------------*/

// SD卡指令表  	 SDIO和SPI的指令都有  
#define SDCard_CMD0    0       //命令0 ，卡复位，复位所有的卡到空闲状态
#define SDCard_CMD2	   2	   //命令2 ，通知所有的卡通过CMD命令返回CID值，因CID值128位，所以返回的响应也为长响应所以用得少。
#define SDCard_CMD3	   3	   //命令3 ，SEND_RELATIVE_ADDR 通知所有卡返回RCA相对地址，也是最常用的地址 RCA相对地址(Relative card address):卡的本地系统地址
#define SDCard_CMD7	   7	   //命令7 ，通过发送RCA相对地址，去选定/取消选定该地址的卡进行操作（读取写入数据）。
#define SDCard_CMD8    8       //命令8 ，SEND_IF_COND 发送SD卡接口条件， 包含主机支持的电压信息， 并询问卡是否支持。
#define SDCard_CMD9    9       //命令9 ，读CSD寄存器数据，CSD的值也为128位。CSD：卡的特定数据(Card Specific Data):卡的操作条件信息
#define SDCard_CMD12   12      //命令12，停止数据传输
#define SDCard_CMD13   13      //命令13，获取状态寄存器的值，在SDIO模式下需要传入RCA
#define SDCard_CMD16   16      //命令16，设置块大小 应返回0x00 对于标准SD卡， 设置块命令的长度， 对于SDHC卡块命令长度固定为512字节。
#define SDCard_CMD17   17      //命令17，读块 需要传入想要读取的地址，对于标准卡， 读取SEL_BLOCK_LEN长度字节的块；对于SDHC卡， 读取512字节的块。
#define SDCard_CMD18   18      //命令18，读多个块，需要传入想要读取的地址，通过CMD12取消
#define SDCard_CMD23   23      //命令23，设置多个块，写入前预先擦除N个block
#define SDCard_CMD24   24      //命令24，向块内写数据，对于标准卡， 写入SEL_BLOCK_LEN长度字节的块；对于SDHC卡，写入512字节的块。这里写512
#define SDCard_CMD25   25      //命令25，写多个块。连续向SD卡写入数据块， 直到被CMD12中断。 每块长度同CMD17。
#define SDCard_CMD41   41      //命令41，应返回0x00 主机要求卡发送它的支持信息(HCS)和OCR寄存器内容。通过HCS位及其响应判断是SDSC还是SDHC卡,HCS（高容量支持）位为1的话为SDHC卡
#define SDCard_CMD55   55      //命令55，应返回0x01 指定下个命令为特定应用命令， 不是标准命令
#define SDCard_CMD58   58      //命令58，读OCR信息

//函数声明              
u8 SDCardReadWriteOneByte(u8 data);                 //底层接口，SPI读写字节函数
u8 SDCardDeviceInit(void);							            //初始化
void SDCardReadData(u8*buf,u32 sector,u32 cnt);		  //读块(扇区)
void SDCardWriteData(u8*buf,u32 sector,u32 cnt);		//写块(扇区)
u8 SDCardSendData(u8*buf,u8 cmd);  									//发送数据包
u8 SDCardRecvData(u8*buf,u16 len);									//接收数据包
u32 GetSDCardSectorCount(void);   					        //读扇区数
#endif

