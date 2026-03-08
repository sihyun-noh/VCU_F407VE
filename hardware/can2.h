#ifndef __CAN2_H_
#define __CAN2_H_

#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_can.h"
#include "stdio.h"



//初始化GPIO,配置CAN
void Init_CAN2_GPIO(void);
//初始化CAN的接收中断
void Init_CAN2_RecvIT(void);
//初始化筛选器
void Init_CAN2_Filter(void);
//初始化CAN（总）
void Init_CAN2();
//配置并发送数据
void CAN2_SetTransmit( CanTxMsg* TxMessage,char *data);
//打印CAN接收的数据
void CAN2_PrintRecvData(CanRxMsg *RxMessage);
//打印CAN发送的数据
void CAN2_PrintSendData(CanTxMsg *TxMessage);

#endif
