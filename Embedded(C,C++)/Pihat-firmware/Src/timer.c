/*
 * timer.c
 *
 *  Created on: Jun 1, 2021
 *      Author: Prakhar Goel
 */

#include"timer.h"
uint32_t tick;
/*
  * @brief This function initializes the timer for precise timing purposes
  * @note  initializes the interrupts for the timer and starts the counter
  * @param  None
  * @retval None
 */
void timer_init(void)
{

 tick=0;
 TIM16->CR1 |=TIM_CR1_URS;
 TIM16->DIER |=TIM_DIER_UIE;
 TIM16->EGR |=TIM_EGR_UG;
 TIM16->CR1|=TIM_CR1_CEN;

 NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

}
/*
  * @brief This function provides delay in intervals of 10 microseconds
  * @note
  * @param  delay needed in multiples of 10 microseconds
  * @retval None
 */
void delay_micro(int us)
{
  uint32_t tick_now = get_tick_micro();

  while(get_tick_micro()-tick_now <(us/10));

}
/*
  * @brief This function returns the tick value with a resolution of 10 microseconds
  * @param  None
  * @retval The tick value
 */
uint32_t get_tick_micro()
{
	return (tick*10);
}

//ISR
/*
  * @brief This callback for the timer interrupt
  * @note  tick is incremented
  * @param  None
  * @retval None
 */
void TIM1_UP_TIM16_IRQHandler()
{
	tick=tick+1;
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	TIM16->SR &= ~TIM_SR_UIF;
}


