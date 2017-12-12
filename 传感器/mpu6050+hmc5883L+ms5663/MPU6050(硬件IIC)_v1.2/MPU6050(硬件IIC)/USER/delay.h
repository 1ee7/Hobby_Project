#ifndef _DELAY__H_
#define _DELAY__H_
#include "stm32f10x.h"
#include "misc.h"


void delay_init(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);

#endif
