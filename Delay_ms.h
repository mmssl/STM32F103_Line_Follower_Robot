#include "stm32f10x.h"                  // Device header

void TIM2Config (void)
{
		/************** STEPS TO FOLLOW *****************
	1. Enable Timer clock
	2. Set the prescalar and the ARR
	3. Enable the Timer, and wait for the update Flag to set
	************************************************/
	
	RCC->APB1ENR |= (1<<0); //Enable TIM2 Clock	
	TIM2->PSC = 72-1; 			//Set PreScaler for TIM2
	TIM2->ARR = 0xffff;			//Set Max ARR value
	TIM2->CR1 |= (1<<0);		//Enable the TIM2
	while(!(TIM2->SR & (1<<0)));	// Wait to be set
}

void delay_us (uint16_t us)
{
	/************** STEPS TO FOLLOW *****************
	1. RESET the Counter
	2. Wait for the Counter to reach the entered value. As each count will take 1 us, 
		 the total waiting time will be the required us delay
	************************************************/
	
	TIM2->CNT &= ~(0xffff);
	while(TIM2->CNT < us);
}

void delay_ms (uint16_t ms)
{
	for(uint16_t i=0;i<ms;i++)
	{
		delay_us(1000); // 1 ms delay
	}
}
