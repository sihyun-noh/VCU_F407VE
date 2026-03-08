#ifndef IO_10_IN_H__
#define IO_10_IN_H__


#include "board.h"
#include <stdio.h>
#include "stm32f4xx.h"                  // Device header
#include "LED.h"
#include "delay.h"
#include "IO_10_Out.h"

void Init_IO_IN(void);
u8 ReadIO_IN_Val(u8 number);

void IO_Configer1(void);
void IO_Configer2(void);
void IO_Configer3(void);
void IO_Configer4(void);
void IO_Configer5(void);
void IO_Configer6(void);
void IO_Configer7(void);
void IO_Configer8(void);
void IO_Configer9(void);
void IO_Configer10(void);

//Âß¼­¿ØÖÆº¯Êý
void IO_LogicCtrl(u8 num);

#endif

