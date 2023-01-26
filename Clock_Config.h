#include "stm32f10x.h"                  // Device header

void initClockPLL(void)
{
	int FLAG = 0;
 /* (1) Test if PLL is used as System clock */
 /* (2) Select HSI as system clock */
 /* (3) Wait for HSI switched */
 /* (4) Disable the PLL */
 /* (5) Wait until PLLRDY is cleared */
 /* (6) Set the PLL multiplier to XX */
 /* (7) Enable the PLL */
 /* (8) Wait until PLLRDY is set */
 /* (9) Select PLL as system clock */
 /* (10) Wait until the PLL is switched on */

		RCC->CFGR &= ~RCC_CFGR_SW;

	 /* (1) Test if PLL is used as System clock */
		if (RCC_CFGR_SWS_PLL && (FLAG==0))
		{			
			FLAG = 1;
		}

			if (FLAG==1){
	 /* (2) Select HSI as system clock */
	
			RCC->CFGR |= RCC_CFGR_SW_HSI;

	 /* (3) Wait for HSI switched */
			RCC->CFGR &= ~RCC_CFGR_SW;
			while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI){}
		
		/* (4) Disable the PLL */

			RCC->CR &= ~RCC_CR_PLLON;
		
		/* (5) Wait until PLLRDY is cleared */
			while((RCC->CR & RCC_CR_PLLRDY) == RCC_CR_PLLON){}
				
		/* (6) Set the PLL multiplier to XX */
			RCC->CFGR |= RCC_CFGR_PPRE1_2;	
			RCC->CFGR &= ~RCC_CFGR_PLLMULL; // Reset the PLLMUL				
			RCC->CFGR |= 	RCC_CFGR_PLLMULL16; // Setting PLLMUL
		
		/* (7) Enable the PLL */	
			RCC->CR |= RCC_CR_PLLON; 
			
		/* (8) Wait until PLLRDY is set */	
			while(!(RCC->CR & RCC_CR_PLLRDY));
		
		/* (9) Select PLL as system clock */
			RCC->CFGR |= RCC_CFGR_SW_PLL;
			
		/* (10) Wait until the PLL is switched on */
			while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL){}	
		}
}
