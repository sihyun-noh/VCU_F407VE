
/*
*************************************************************************
*                             ������ͷ�ļ�
*************************************************************************
*/

#include "rtthread.h"
#include "main.h"
#include "board.h"
#include "spi.h"

#include "vcu_gateway.h"

/*
*************************************************************************
*                               ����
*************************************************************************
*/

u8 num = 2; // ��������ѡ���ĵ�����Ƕ�����Ĳ�����

uint16_t RecData = 0;
extern int8_t flag;
extern int8_t RS485_3_flag;
extern int8_t RS232_2_flag;
extern int8_t RS232_1_flag;

// extern wiz_NetInfo gWIZNETINFO;

// eeprom��д����
u8 t = 1, data = 0;

/*
*************************************************************************
*                             ��������
*************************************************************************
*/

/*
*************************************************************************
*                             main ����
*************************************************************************
*/

/**
 * @brief  ������
 * @param  ��
 * @retval ��
 */

static struct rt_messagequeue sbus_rx_mq;
static volatile int a = 10;

int main(void) {
  rt_kprintf("\r\n AGMO START MAIN\r\n");
  rt_hw_console_output("\nAGMO\n");

	vcu_gateway_init();
	//bsp_Sbus_thread_AGMO(1);
	
	
  // bsp_battery_thread();				//ad�ɼ�����
  // bsp_MPU6050_thread();				//����̬
  // bsp_TH_thread();   					//����ʪ��

  // bsp_motor1_thread(111);				//СԲ���������
  // bsp_EC800_USART_thread();			//Զ������ģ��

  // bsp_Sbus_thread(4); // ң��������С������

  // bsp_Modbus_thread();				//Modbus�Խ���λ�����߳�

  // bsp_w5500_thread();					//����ͨ��ģ��
  // bsp_Ee_thread();					//EEPROMģ�����

  Init_LED();

  u16 FreqValue = 0;
  u32 DutyValue = 0;

#if 0

/***************************************************DAC������Ҳ�**********************************************/
	printf("\r\n DAC������̣�������Ҳ�\r\n");	
	printf("\r\n ʹ��ʾ������⿪�����PA4��PA5���ţ��ɲ�����Ҳ�\r\n ");	
	/*��ʼ��DAC����ʼDACת��,ʹ��ʾ�������PA4/PA5���ɹ۲쵽���Ҳ�*/
	DAC_Mode_Init();

#endif

  // MotorEnable();
  // RCC_ClocksTypeDef RCC_Clocks;
  //	if(MPU6050_CheckDevice(MPU6050_ADDRESS) == 0)
  //	{
  //		printf("��⵽MPU6050��\n");
  //	}
  // while(1)
  //	{

#if 0
/****************************************************RS485_4���Գ���****************************************************/
		
//		RS485_4_TxMode();													//����ģʽ
//		printf("hello,world");
////		RS485_4_SendByte(65);
//		Delay_Ms(1);														//�ȴ��������
//		RS485_4_RxMode();													//��Ϊ����ģʽ
//		Delay_Ms(3000);
		if(flag == 1)
		{
			RS485_4_TxMode();
			RS485_4_SendByte(RecData);
			Delay_Ms(1);
			RS485_4_RxMode();
			flag =0;
		}

#endif

#if 0
/****************************************************RS485_3���Գ���****************************************************/
		
		RS485_3_TxMode();													//����ģʽ
		RS485_3_SendByte(65);
		Delay_Ms(1);														//�ȴ��������
		RS485_3_RxMode();													//��Ϊ����ģʽ
		Delay_Ms(3000);
		if(RS485_3_flag == 1)
		{
			RS485_3_TxMode();
			RS485_3_SendByte(RecData);
			Delay_Ms(1);
			RS485_3_RxMode();
			RS485_3_flag =0;
		}

#endif

#if 0
/****************************************************RS232_2���Գ���****************************************************/
		
		
//		RS232_2_SendByte(65);
		printf("hello world\n");
		Delay_Ms(1000);
		if(RS232_2_flag == 1)
		{
			RS232_2_SendByte(RecData);
			RS232_2_flag =0;
		}

#endif

#if 0
/************************************I2C����(AT24C02)��д��������*************************************/

printf("\r\n ����һ��I2C����(AT24C02)��д�������� \r\n");

//	if(ee_Test() == 1)
//	{
//		LED_ON();
//		Delay_Ms(1000);
//		LED_OFF();
//	}
//	else
//	{
//		LED_OFF();
//	}

	data =6;
	printf("д�������Ϊ��data = %#x\n",data);
	EEPROM_WriteByteData(3,data);
	Delay_Ms(10);
	t = EEPROM_ReadByteData(3);
	printf("����������Ϊ��t = %#x\n",t);
#endif

#if 0		
/************************************** һ·pwmʵ�ֺ�����  PWM����  ******************************************/
		
//		for(int i = 0;i<100;i++)
//		{
//			TIM_SetCompare1(TIM11, i);
//			Delay_Ms(10);
//		}
//		for(int j = 100;j>0;j--)
//		{
//			TIM_SetCompare1(TIM11, j);
//			Delay_Ms(10);
//		}
		TIM_SetCompare1(TIM11, 0);

#endif

#if 0		
/************************************** ���벶����Գ���  ******************************************/
	Delay_Ms(1000);
	printf("hello world!\n");
	FreqValue = Get_Input2Freq()-1;
	printf("���ܵ����źŵ�Ƶ��Ϊ��%dHZ\n",FreqValue);
//	DutyValue = Get_Input1Duty() + 1;
//	printf("���ܵ����źŵ�ռ�ձ�Ϊ��%d%%\n",DutyValue);

#endif

#if 0
/******************************************************CAN2�ػ�����*******************************************************/

			Delay_Ms(3000);
			
			printf("ѭ������3��һ�Σ�\n");
			//����Ҫ���͵�����
			CAN2_SetTransmit(&TxMessage,Start_Data);
			
			//�ѷ������ݵķ�������ŷ��س����������������᷵�� CAN_TxStatus_NoMailBox
			Mail_Box = CAN_Transmit(CAN2,&TxMessage);											//�����ݷ���
			
			//�������Ŵ���
			if(Mail_Box != CAN_TxStatus_NoMailBox)
			{
				
				//�ȴ�ֱ�������佫���ݷ��͵�CAN�շ������
				while(CAN_TransmitStatus(CAN2,Mail_Box)!=CAN_TxStatus_Ok );//�ȴ�CAN�շ������͵�������ϣ���ʹ��CAN_TransmitStatus�鿴״̬
				
			}
			else
			{
				printf("no mail_box !\n");
			}

			
			Delay_Ms(10);
			//can_delay(10000);//�ȴ�������ϣ���ʹ��CAN_TransmitStatus�鿴״̬
			printf("����CAN2����Ϊ��");
			CAN2_PrintSendData(&TxMessage);

			putchar(10);
			if(CAN_Flag == 1)
			{
				printf("����CAN2����׼ȷ����\n");
				printf("���յ�CAN2����Ϊ��");
				//�����ݴ�ӡ��
				CAN2_PrintRecvData(&RxMessage);
				CAN_Flag = 0;
			}

#endif

#if 0
/*************************************************** ��дSD�� **************************************************/
	if(0 != SDCardDeviceInit())
	{
		printf("��ʼ��ʧ�ܣ�\n");
	}
	else
	{
		printf("��ʼ���ɹ���\n");
	}
	Delay_Ms(1000);
#endif

#if 0
/*************************************************** �޸���Ƶ ��ȡ���������ϵ�Ƶ�� **************************************************/
	SystemCoreClockUpdate();										//�޸���Ƶ�����ڸ���ϵͳʱ�ӵ�ֵ
	printf("SystemCoreClock:%d\n",SystemCoreClock);
	
	RCC_GetClocksFreq(&RCC_Clocks);	
	printf("RCC_Clocks.HCLK_Frequency :%d\n",RCC_Clocks.HCLK_Frequency);
	printf("RCC_Clocks.PCLK1_Frequency :%d\n",RCC_Clocks.PCLK1_Frequency);
	printf("RCC_Clocks.PCLK2_Frequency :%d\n",RCC_Clocks.PCLK2_Frequency);
	printf("RCC_Clocks.SYSCLK_Frequency :%d\n",RCC_Clocks.SYSCLK_Frequency);
	Delay_ms(1000);
#endif

  //	}

  // return 0;
}
