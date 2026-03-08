#ifndef IO_10_Out_H__
#define IO_10_Out_H__

#include <stdio.h>
#include "stm32f4xx.h"                  // Device header

void Init_IO_Out(void);
void OpenCloseIO_Out(u8 number,BitAction BitVal);

#endif