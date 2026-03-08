#include "IO_10_IN.h"


u8 IN_IO_Number;

u8 Out_IO_Value1,Out_IO_Value2,Out_IO_Value3,Out_IO_Value4,Out_IO_Value5,
	Out_IO_Value6,Out_IO_Value7,Out_IO_Value8,Out_IO_Value9,Out_IO_Value10;

u8 IN_Delay1,IN_Delay2,IN_Delay3,IN_Delay4,IN_Delay5,
	IN_Delay6,IN_Delay7,IN_Delay8,IN_Delay9,IN_Delay10;

u8 Out_Delay1,Out_Delay2,Out_Delay3,Out_Delay4,Out_Delay5,
	Out_Delay6,Out_Delay7,Out_Delay8,Out_Delay9,Out_Delay10;



/*
功能：初始化十路IO口开关量
*/
void Init_IO_IN(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOD,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;												//配置为输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;												//配置为推完
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;													//配置为PA0口
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;											//配置为浮空输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_10;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_8|
	GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
}




u8 ReadIO_IN_Val(u8 number)
{
	BitAction BitVal = 0;
	switch(number)
	{
		case 1:
			BitVal = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11);
			break;
		case 2:
			BitVal = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1 );
			break;
		case 3:
			BitVal = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13);
			break;
		case 4:
			BitVal = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12);
			break;
		case 5:
			BitVal = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9 );
			break;
		case 6:
			BitVal = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_8 );
			break;
		case 7:
			BitVal = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_15);
			break;
		case 8:
			BitVal = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_10);
			break;
		case 9:
			BitVal = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7 );
			break;
		case 10:
			BitVal = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_10);
			break;
		default:
			printf("input eorr!");
			break;
	}
	return BitVal;
}


//输入开关量1的配置函数
void IO_Configer1(void)
{
	if(ReadIO_IN_Val(IN_IO_Number) == 1)
	{
		Delay_s(IN_Delay1);
		OpenCloseIO_Out(Out_IO_Value1,1);
		Delay_s(Out_Delay1);
		if(Out_Delay1)
		{
			OpenCloseIO_Out(Out_IO_Value1,0);
		}
	}
}


//输入开关量2的配置函数
void IO_Configer2(void)
{
	if(ReadIO_IN_Val(IN_IO_Number) == 1)
	{
		Delay_s(IN_Delay2);
		OpenCloseIO_Out(Out_IO_Value2,1);
		Delay_s(Out_Delay2);
		if(Out_Delay2)
		{
			OpenCloseIO_Out(Out_IO_Value2,0);
		}
	}
}



//输入开关量3的配置函数
void IO_Configer3(void)
{
	if(ReadIO_IN_Val(IN_IO_Number) == 1)
	{
		Delay_s(IN_Delay3);
		OpenCloseIO_Out(Out_IO_Value3,1);
		Delay_s(Out_Delay3);
		if(Out_Delay3)
		{
			OpenCloseIO_Out(Out_IO_Value3,0);
		}
	}
}



//输入开关量4的配置函数
void IO_Configer4(void)
{
	if(ReadIO_IN_Val(IN_IO_Number) == 1)
	{
		Delay_s(IN_Delay4);
		OpenCloseIO_Out(Out_IO_Value4,1);
		Delay_s(Out_Delay4);
		if(Out_Delay4)
		{
			OpenCloseIO_Out(Out_IO_Value4,0);
		}
	}
}


//输入开关量5的配置函数
void IO_Configer5(void)
{
	if(ReadIO_IN_Val(IN_IO_Number) == 1)
	{
		Delay_s(IN_Delay5);
		OpenCloseIO_Out(Out_IO_Value5,1);
		Delay_s(Out_Delay5);
		if(Out_Delay5)
		{
			OpenCloseIO_Out(Out_IO_Value5,0);
		}
	}
}


//输入开关量6的配置函数
void IO_Configer6(void)
{
	if(ReadIO_IN_Val(IN_IO_Number) == 1)
	{
		Delay_s(IN_Delay6);
		OpenCloseIO_Out(Out_IO_Value6,1);
		Delay_s(Out_Delay6);
		if(Out_Delay6)
		{
			OpenCloseIO_Out(Out_IO_Value6,0);
		}
	}
}


//输入开关量7的配置函数
void IO_Configer7(void)
{
	if(ReadIO_IN_Val(IN_IO_Number) == 1)
	{
		Delay_s(IN_Delay7);
		OpenCloseIO_Out(Out_IO_Value7,1);
		Delay_s(Out_Delay7);
		if(Out_Delay7)
		{
			OpenCloseIO_Out(Out_IO_Value7,0);
		}
	}
}


//输入开关量8的配置函数
void IO_Configer8(void)
{
	if(ReadIO_IN_Val(IN_IO_Number) == 1)
	{
		Delay_s(IN_Delay8);
		OpenCloseIO_Out(Out_IO_Value8,1);
		Delay_s(Out_Delay8);
		if(Out_Delay8)
		{
			OpenCloseIO_Out(Out_IO_Value8,0);
		}
	}
}


//输入开关量9的配置函数
void IO_Configer9(void)
{
	if(ReadIO_IN_Val(IN_IO_Number) == 1)
	{
		Delay_s(IN_Delay9);
		OpenCloseIO_Out(Out_IO_Value9,1);
		Delay_s(Out_Delay9);
		if(Out_Delay9)
		{
			OpenCloseIO_Out(Out_IO_Value9,0);
		}
	}
}


//输入开关量10的配置函数
void IO_Configer10(void)
{
	if(ReadIO_IN_Val(IN_IO_Number) == 1)
	{
		Delay_s(IN_Delay10);
		OpenCloseIO_Out(Out_IO_Value10,1);
		Delay_s(Out_Delay10);
		if(Out_Delay10)
		{
			OpenCloseIO_Out(Out_IO_Value10,0);
		}
	}
}



//逻辑控制函数
void IO_LogicCtrl(u8 num)
{
	switch(IN_IO_Number)
	{
		case 1:
			IO_Configer1();
			break;
		case 2:
			IO_Configer2();
			break;
		case 3:
			IO_Configer3();
			break;
		case 4:
			IO_Configer4();
			break;
		case 5:
			IO_Configer5();
			break;
		case 6:
			IO_Configer6();
			break;
		case 7:
			IO_Configer7();
			break;
		case 8:
			IO_Configer8();
			break;
		case 9:
			IO_Configer9();
			break;
		case 10:
			IO_Configer10();
			break;
		default:
			printf("input eorr!");
	}
}
