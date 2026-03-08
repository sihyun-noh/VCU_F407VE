#ifndef __PWM_IN_H_
#define __PWM_IN_H_

#include "stm32f4xx.h"                  // Device header
#include "delay.h"
#include "led.h"

void Init_PWM_IN1(void);
void Init_PWM_IN2(void);
void Init_PWM_IN3(void);
void Init_PWM_IN4(void);
void Init_PWM_IN5(void);

u16 Get_Input1Freq();
u16 Get_Input2Freq();
u16 Get_Input3Freq();
u16 Get_Input4Freq();
u16 Get_Input5Freq();

u32 Get_Input1Duty();
u32 Get_Input2Duty();
u32 Get_Input3Duty();
u32 Get_Input4Duty();
u32 Get_Input5Duty();

#endif
