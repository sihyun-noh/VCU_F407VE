#include "LED.h"

void Init_LED(void)
{
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_WriteBit(GPIOB, GPIO_Pin_9|GPIO_Pin_8,1);
}

void LED_ON(void)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_9|GPIO_Pin_8,0);
}

void LED_OFF(void)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_9|GPIO_Pin_8,1);
}

