#ifndef __CLOCK_H
#define __CLOCK_H

#include "stm32f10x.h"  // Device header

static inline void initSystemClock(void)
{

  // (1) Turn on HSI if off
  if(!(RCC->CR & (1<<0))) 
  {
    RCC->CR |= (1<<0);            // HSION=1
    while(!(RCC->CR & (1<<1)));   // Wait HSIRDY=1
  }
  
  // (2) Switch to HSI if not already
  if((RCC->CFGR & (3<<2)) != 0) 
  {
    RCC->CFGR &= ~(3<<0);                 // SW=00 => HSI
    while((RCC->CFGR & (3<<2)) != 0);     // Wait SWS=00
  }
  
  // (3) Disable PLL if on
  if(RCC->CR & (1<<24))
  {
    RCC->CR &= ~(1<<24);          // PLL off
    while(RCC->CR & (1<<25));     // Wait PLLRDY=0
  }
  
  // (4) Configure PLL => x9
  RCC->CFGR &= ~(0xF<<18);        // Reset PLLMUL bits
  RCC->CFGR |=  (7<<18);          // PLLMUL=9 => 0111 => shift 18 => 7<<18
  // For HSI/2 => PLLSRC=0
  
  // (5) Enable PLL
  RCC->CR |= (1<<24);             // PLLON
  while(!(RCC->CR & (1<<25)));    // Wait PLLRDY=1
  
  // (6) Switch system clock to PLL
  RCC->CFGR &= ~(3<<0);           // Clear SW
  RCC->CFGR |=  (2<<0);           // SW=10 => PLL
  while((RCC->CFGR & (3<<2)) != (2<<2)); // Wait SWS=10 => PLL used
}

#endif