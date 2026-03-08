#ifndef AD_H__
#define AD_H__

#include "stm32f4xx.h"                  // Device header
#include <stdio.h>
#include <string.h>
#include "main.h"


extern uint16_t AD_Value[5] ;

void Init_AD(void);
uint16_t Get_AD_Value(void);

int bsp_battery_thread(void);

#endif
