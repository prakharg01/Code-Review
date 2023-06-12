/*
 * timer.h
 *
 *  Created on: Jun 1, 2021
 *      Author: Prakhar Goel
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include<stdint.h>
#include"stm32g474xx.h"

void timer_init(void);
void delay_us(int us);
uint32_t get_tick_micro(void);
void TIM1_UP_TIM16_IRQHandler(void);

#endif /* INC_TIMER_H_ */

