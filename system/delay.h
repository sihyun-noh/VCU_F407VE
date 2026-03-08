#ifndef __DELAY_H
#define __DELAY_H
 
 
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_rcc.h"
#include "core_cm4.h"
#include "stm32f4xx_it.h"
#include "misc.h"
 
 
 
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
 
void Delay_Init(void);					//延时系统初始化
void Delay_Us(uint16_t us);				//微秒延时
void Delay_Ms(u16 ms);				//毫秒延时
void Delay_Us_2(u32 us);				//微秒延时，范围更大
 
void Delay_us(uint32_t xus);

void Delay_ms(uint32_t xms);

void Delay_s(uint32_t xs);
 
 
#endif /* __DELAY_H */
