#include "stm32f10x.h"                  // Device header
#include "Delay_ms.h"
#include "Clock_Config.h"
#include "math.h"

/*Private Variables----------------------------------------------------------*/
/*Begin*/
uint16_t  a,sum;
int position=0;
int summ = 0;
int sharp = 0;
int i;
int read = 0;
int idle = 0;
int P, I, D, R;
double Kp = 0.025; // fast turn
double Kd = 0.038;		// control while turning
double Ki = 0.00001; // oscilation
int last_error = 0;
int error = 0;
/*constant values---------------*/
const uint8_t MaxSpeedRight = 150;
const uint8_t MaxSpeedLeft = 150;
const uint8_t SpeedRight = 100;
const uint8_t SpeedLeft = 100;
/*------------------------------*/
int errors[9] = {0,0,0,0,0,0,0,0,0};
/*End------------------------------------------------------------------------*/

/*Private Functions----------------------------------------------------------*/
/*Begin*/
void initGPIO (void);
void QTR8_config (void);
int QTR8_Read (void);
void TIM4_PWM (void);
void TIM3_PWM (void);
void motor_control (int motorR, int motorL);
void sharp_turn(void);
void PID_error (void);
/*End------------------------------------------------------------------------*/

void initGPIO (void)
{
	RCC->APB2ENR |= (1<<0);							// Enable AF clock
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;	// Enable port A clock
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	// Enable port B clock
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // Enable port C clock
	
	AFIO->MAPR &= ~(1<<12);
	AFIO->MAPR |= (2<<10);
  	GPIOA->CRL &= ~(0xffffffff);
	GPIOB->CRH &= ~(0xffffffff);
	GPIOB->CRL &= ~(0xffffffff);
	GPIOC->CRH &= ~(0xffffffff);
	
	GPIOC->IDR |= (2<<26);
	GPIOB->CRL |= (3<<0) | (2<<2) | (3<<4) | (2<<6); 
	GPIOB->CRH |= (3<<0) | (2<<2) | (3<<4) | (2<<6); //PA8-9-10-11 config
	
	GPIOB->CRH |= GPIO_CRH_MODE10;
}

void TIM4_PWM (void)
{
	RCC->APB1ENR |= (1<<2); //Enable TIM4 Clock	
	TIM4->PSC = 0; 			//Set PreScaler for TIM4
	TIM4->ARR = 25000;			//Set ARR value
	TIM4->CNT &= ~(0xffff); //Reset count value
	TIM4->CCMR2 |= (6<<4) | (6<<12); // set PWM mode
	TIM4->CCER |= (1<<8) | (1<<12);		//Enable CH1 as output
	TIM4->CCR3 = 0;
	TIM4->CCR4 = 0;
	TIM4->CR1 |= (1<<0);					//Enable the TIM1
	while(!(TIM4->SR & (1<<0)));	// Wait to be set
}

void TIM3_PWM (void)
{
	RCC->APB1ENR |= (1<<1); //Enable TIM3 Clock	
	TIM3->PSC = 0; 			//Set PreScaler for TIM4
	TIM3->ARR = 25000;			//Set ARR value
	TIM3->CNT &= ~(0xffff); //Reset count value
	TIM3->CCMR2 |= (6<<4) | (6<<12);
	TIM3->CCER |= (1<<8) | (1<<12);		//Enable CH1 as output
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;
	TIM3->CR1 |= (1<<0);					//Enable the TIM1
	while(!(TIM3->SR & (1<<0)));	// Wait to be set
}

void QTR8_config (void)
{

	GPIOB->ODR |= (1<<10); // set LEDon high
	
	GPIOA->CRL |= (3<<28) | (3<<24) | (3<<20) | (3<<16) | (3<<12) | (3<<8) | (3<<4) | (3<<0);  // set pin as output
	GPIOA->ODR |= 0xffff;		// set pin as high
	delay_us(12);			
	GPIOA->CRL &= ~(3<<28) & ~(3<<24) & ~(3<<20) & ~(3<<16) & ~(3<<12) & ~(3<<8) & ~(3<<4) & ~(3<<0); // set pin as input
	GPIOA->CRL |= (2<<30)  | (2<<26) | (2<<22) | (2<<18) | (2<<14) | (2<<10) | (2<<6) | (2<<2);	// set pin with pull-up/pull-down  
	delay_ms(6);	
	
}

void motor_control (int motorR, int motorL) 
{

		if (motorL<0)
		{
		 TIM3->CCR3 = (100*motorL);	 // PB1
		 TIM3->CCR4 = 0;	// PB0
		}
		else 
		{
		 TIM3->CCR3 = 0;	//PB0
		 TIM3->CCR4 = (100*motorL);
		}
		if (motorR<0)
		{
		 TIM4->CCR3 = (100*motorR);	//PB9
		 TIM4->CCR4 = 0;	
		}
		else 
		{
		 TIM4->CCR4 = (100*motorR);	//PB8
		 TIM4->CCR3 = 0;
		} 

}

void sharp_turn()
{
	if (idle < 50)
		{
			motor_control(0,0);
			if (sharp == 1)
			{
				motor_control(120,0);	
			}
			else 
			{
				motor_control(0,120);
			}
	}
}


int QTR8_Read (void)
{
	int b =0;
	int pos=0;
	QTR8_config();
	
		for (i=0;i<8;i++)
			{			
				if(((GPIOA->IDR & (1<<i))==(1<<i)))					
			{		
				pos+=1000*i;
				b++;				
			}	
		}

		sum = (GPIOA->IDR & (0xff)); // check IDR 
		
		if (((GPIOA->IDR & (1<<0))==(1<<0)))
		{
		sharp = 1;
		}else if(((GPIOA->IDR & (1<<7))==(1<<7)))
		{
		sharp = 0;
		}
		
		read = b;
		
		if (read == 0)
		{
		idle++;
		}
		else
		{
		idle = 0;
		}
		
		position = pos/b;
		error = 3500 - position;
		
		summ=0;
		for ( a = 0; a<5 ; a++)
			{
				errors[a] = errors[a-1];
				errors[0] = error;
				if (errors[a]<0)
				summ += errors[a];
			}
			
			GPIOB->ODR &= ~(1<<10); // set LEDon low
			
	return pos/b;
}

void PID_error (void)
{

	QTR8_Read();
	error = 3500 - position;
	
	P = error;
	D = error-last_error;
	I = summ; // sum of the errors
	last_error  = error; //store the error
	
	int adjustment = ((Kp*P) + (Kd*D) + (I*Ki)); // PID calculation 
	
	int motorR = (SpeedRight + adjustment);
	int motorL = (SpeedLeft - adjustment);
	
		
		if(sum == 255) // motor will not be powered if sensor all black or in this case car not in the track
		{
		 motorL  = 0;	
		 motorR = 0;	
		}

		if (motorL > MaxSpeedLeft)
		{
		motorL = MaxSpeedLeft;
		}
		if (motorR > MaxSpeedRight)
		{
		motorR = MaxSpeedRight;
		}
		
		if (read==0)
		{
		sharp_turn();
		}
		else
		{
		motor_control(motorR, motorL);
		}
		
}


int main()
 {
	SystemInit();
	TIM2Config();
	initGPIO();
	TIM4_PWM();
	TIM3_PWM();
	
	while(1)
{
	
		PID_error();
	
}
return 0;
}
