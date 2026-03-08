
#include "IO_10_Out.h"



/*
功能：初始化十路IO口开关量
*/
void Init_IO_Out(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOE,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;												//配置为输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;												//配置为
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;													//配置为PA0口
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;											//配置为浮空输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|
	GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_15;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
}


/*
功能：使能或失能开关量输出
number：指定哪一个开关量输出    		取值为1~10
BitVal：使能或者失能开关量输出       取值1使能，0失能/Set or Reset
*/
void OpenCloseIO_Out(u8 number,BitAction BitVal)
{
	switch(number)
	{
		case 1:
			GPIO_WriteBit(GPIOE, GPIO_Pin_0,BitVal);
			break;
		case 2:
			GPIO_WriteBit(GPIOE, GPIO_Pin_1,BitVal);
			break;
		case 3:
			GPIO_WriteBit(GPIOE, GPIO_Pin_2,BitVal);
			break;
		case 4:
			GPIO_WriteBit(GPIOE, GPIO_Pin_3,BitVal);
			break;
		case 5:
			GPIO_WriteBit(GPIOE, GPIO_Pin_4,BitVal);
			break;
		case 6:
			GPIO_WriteBit(GPIOE, GPIO_Pin_5,BitVal);
			break;
		case 7:
			GPIO_WriteBit(GPIOE, GPIO_Pin_6,BitVal);
			break;
		case 8:
			GPIO_WriteBit(GPIOA, GPIO_Pin_0,BitVal);
			break;
		case 9:
			GPIO_WriteBit(GPIOE, GPIO_Pin_15,BitVal);
			break;
		case 10:
			GPIO_WriteBit(GPIOC, GPIO_Pin_5,BitVal);
			break;
		default:
			printf("input eorr!");
	}
}


