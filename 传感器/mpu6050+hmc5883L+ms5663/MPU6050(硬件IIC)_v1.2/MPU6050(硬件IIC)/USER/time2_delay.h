#ifndef __TIME2_DELAY_H
#define __TIME2_DELAY_H
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"
//#include "stm32f10x_lib.h"
#define START_TIME  time=0;RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);TIM_Cmd(TIM2, ENABLE)
#define STOP_TIME  TIM_Cmd(TIM2, DISABLE);RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE)

void TIM2_NVIC_Configuration(void);
void TIM2_Configuration(void);


void TIM2_init(void);
//void delay_us(uint32_t n_usec);
//void delay_ms(uint32_t n_msec);

void delay_us(uint16_t times);
void delay_ms(uint16_t n_msec);

//volatile u32 time;
#endif

