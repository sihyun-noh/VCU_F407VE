/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "CAN.h"
#include "stdio.h"
#include "SBUS_UART.h"
#include "SBUS_AGMO.h"
#include "CAN_AGMO.h"
#include "vcu_gateway.h"
//外部引用CAN的接收中断标志位

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
//void HardFault_Handler(void)
//{
//  /* Go to infinite loop when Hard Fault exception occurs */
//  while (1)
//  {
//  }
//}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

///**
//  * @brief  This function handles PendSVC exception.
//  * @param  None
//  * @retval None
//  */
//void PendSV_Handler(void)
//{
//}

///**
//  * @brief  This function handles SysTick Handler.
//  * @param  None
//  * @retval None
//  */
//void SysTick_Handler(void)
//{
//	
//}

extern u8 CAN_Flag ;
extern CanRxMsg RxMessage;

//CAN的接收中断
void CAN1_RX0_IRQHandler(void)
{
	A_CAN1_RX0_IRQHandler(gateway_can_rx_push_isr_from_rxmsg);
#if 0
	printf("心跳为：\n");
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)==SET)
	{
		CAN_Receive(CAN1,CAN_FIFO0,&RxMessage);
		if((RxMessage.ExtId == 0x18ff2100)&&(RxMessage.IDE == CAN_Id_Extended)&&(RxMessage.DLC == 8))
		{
			CAN1_PrintRecvData(&RxMessage);
//			CAN_Flag = 1;			//接收成功
		}
		else
		{
			CAN_Flag = 0;			//接收失败
		}
	}	
#endif
	
}

void CAN2_RX0_IRQHandler(void)
{
	printf("进入了CAN2的接收中断！\n");
	if(CAN_GetITStatus(CAN2,CAN_IT_FMP0)==SET)
	{
		CAN_Receive(CAN2,CAN_FIFO0,&RxMessage);
		if((RxMessage.ExtId == 0x07000001)&&(RxMessage.IDE == CAN_Id_Extended)&&(RxMessage.DLC == 8))
		{
			CAN_Flag = 1;			//接收成功
		}
		else
		{
			CAN_Flag = 0;			//接收失败
		}
	}	
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/


// SBUS IRQ handler
void BSP_USART6_IRQHandler(void)
{
	A_SBUS_USART6_IRQHandler();
#ifdef 0
//#ifdef BSP_USART6_NORMAL_MODE
	if(USART_GetITStatus(BSP_USART6,USART_IT_IDLE)!=RESET)
	{		
      BSP_USART6_DMA_Rx_Data();       /* 释放一个信号量，表示数据已接收 */
      USART_ReceiveData(BSP_USART6); /* 清除标志位 */
	}	 
//#endif
	
//#ifdef BSP_USING_SBUS

	static char recive_index = 0;
	if(USART_GetITStatus(BSP_USART6,USART_IT_RXNE)!=RESET)
	{
		if(recive_index < BSP_USART6_RBUFF_SIZE)
				USART6_Rx_Buf[recive_index++] = USART_ReceiveData(BSP_USART6);
		else
		{
			memset(USART6_Rx_Buf,0,recive_index);
			recive_index = 0;
		}
				
			
		if(USART6_Rx_Buf[0] != 0x0F)
		{
			USART6_Rx_Buf[0] = 0x00;
			recive_index = 0;
		}
	}

	else if(USART_GetITStatus(BSP_USART6,USART_IT_IDLE)!=RESET)
	{		
		USART_ReceiveData(BSP_USART6);											/* 清除标志位 */
		if(USART6_Rx_Buf[1] == 0x0f)
		{
			memset(USART6_Rx_Buf,0,recive_index);
		}
		else
		{
			if(USART6_Rx_Buf[0] == 0x0F && recive_index == 25)	//接受完一帧数据
			{
				//D-bug
//				for(char i = 0;i<25;i++)
//				{
//					rt_kprintf("%d ",USART6_Rx_Buf[i]);
//				}
				//putchar(10);
				if(update_sbus((uint8_t *)USART6_Rx_Buf) == 1)
				{
					//转换后接收到的数据(遥控器连接上才进行转换)
					//rt_kprintf("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d \n",SBUS_CH.CH1,SBUS_CH.CH2,SBUS_CH.CH3,SBUS_CH.CH4,SBUS_CH.CH5,SBUS_CH.CH6,SBUS_CH.CH7,SBUS_CH.CH8,SBUS_CH.CH9,SBUS_CH.CH10,SBUS_CH.CH11,SBUS_CH.CH12,SBUS_CH.CH13,SBUS_CH.CH14,SBUS_CH.CH15,SBUS_CH.CH16);
					
				}
				//BSP_USRT3_SEND_SEM();	
			}

			memset(USART6_Rx_Buf,0,recive_index);
		}
		recive_index = 0;
	}	 
//	
//	if(USART_GetITStatus(BSP_USART6,USART_IT_RXNE)!=RESET)
//	{
//		USART6_Rx_Buf[recive_index] = USART_ReceiveData(BSP_USART6);
//		if (recive_index == 0 && USART6_Rx_Buf[recive_index] != 0x0F) //帧头不对，丢掉
//		{
//			recive_index = 0;
//			USART6_Rx_Buf[0] =0;
//		}
//		else
//		{
//			recive_index++;
//			if (recive_index > BSP_SBUSS_RBUFF_SIZE)
//				recive_index = 0;  ///接收数据错误,重新开始接收
//			
//			if (USART6_Rx_Buf[0] == 0x0F && USART6_Rx_Buf[24] == 0x00 && recive_index == 25)	//接受完一帧数据
//			{
//				if(update_sbus((uint8_t *)USART6_Rx_Buf) == 1)
//						BSP_USRT3_SEND_SEM();	
//				
//				memset(USART6_Rx_Buf,0,BSP_USART6_RBUFF_SIZE);
//				recive_index = 0;
//			}
//		}
//	}
	

//#endif
#endif

}






/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
