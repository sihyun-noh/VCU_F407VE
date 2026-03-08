/**
  ******************************************************************************
  * @file    spi.h
  * $Author: wdluo $
  * $Revision: 17 $
  * $Date:: 2012-07-06 11:16:48 +0800 #$
  * @brief   SPI驱动函数声明.
  ******************************************************************************
  * @attention
  *
  *<h3><center>&copy; Copyright 2009-2012, ViewTool</center>
  *<center><a href="http:\\www.viewtool.com">http://www.viewtool.com</a></center>
  *<center>All Rights Reserved</center></h3>
  * 
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H
#define __SPI_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

/* Exported Functions --------------------------------------------------------*/

void SPI_Configuration(void);
void SPI_WriteByte(uint8_t TxData);
uint8_t SPI_ReadByte(void);
void SPI_CrisEnter(void);
void SPI_CrisExit(void);
void SPI_CS_Select(void);
void SPI_CS_Deselect(void);

void w5500_test(void);
void platform_init(void);								// 初始化相关的主机外设
void network_init(void);								// 初始化网络信息并显示

static void w5500_thread_entry(void *parameter);
int bsp_w5500_thread(void);


#endif /* __SPI_H */

/*********************************END OF FILE**********************************/
