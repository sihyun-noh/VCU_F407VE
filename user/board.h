
#ifndef __BOARD_H__
#define __BOARD_H__
#include "stm32f4xx.h"                  // Device header
#include <rthw.h>
#include <rtthread.h>


void rt_hw_board_init(void);
 
void rt_hw_us_delay(rt_uint32_t us);

void rt_hw_console_output(const char *str);
 
#endif
