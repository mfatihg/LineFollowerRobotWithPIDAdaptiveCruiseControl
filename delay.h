#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"  // Device header

/* --- Initialize Timer2 for microsecond Delay --- */
static inline void initTimer2ForDelay(void)
{
  RCC->APB1ENR |= (1<<0);   // Enable TIM2 clock
  TIM2->PSC = 72 - 1;       // Prescale 72 => 1 MHz => 1 tick = 1 us
  TIM2->ARR = 0xFFFF;       // Max ARR
  TIM2->CR1 |= (1<<0);      // Enable TIM2
  while(!(TIM2->SR & (1<<0))); // Wait for update event
}

/* --- Microsecond Delay --- */
static inline void delay_us(uint16_t us)
{
  TIM2->CNT = 0;                 // Reset counter
  while(TIM2->CNT < us);         // Wait until it reaches 'us'
}

/* --- Millisecond Delay --- */
static inline void delay_ms(uint16_t ms)
{
  for(uint16_t i=0; i<ms; i++)
    delay_us(1000);              // 1 ms => 1000 us
}

#endif