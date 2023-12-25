#include "stm32f10x.h"                  // Device header
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

void delay(uint32_t dlyTicks);
/* Counts 1ms timeTicks */
volatile uint32_t msTicks = 0;
void SysTick_Handler(void);
void Clock_Init(void);
void TIM4_Init(void);
void TIM2_Init(void);
uint16_t count = 0;
uint16_t pre_count = 0;
int val = 0;



void display(uint16_t number);

uint16_t disp[4] = {0,0,0,0};
uint16_t digit[10] = {0x0E07,0x0F3F,0x0E4B,0x0E1B,0x0F33,0x0E93,0x0E83,0x0E3F,0x0E03,0x0E13};
                  //     0      1      2      3      4     5      6      7      8      9
double adc_value;
void ADC1_2_IRQHandler(void){
	// read ADC value and clear EOC flag
	adc_value = (float)(ADC1->DR)/4095 * 3.3;
}

void ADC_Init(){
	RCC->APB1ENR |= (1U<<0);
	// configure the ADC
	ADC1->CR2 &= ~(1<<1);							// Single mode
	//ADC1->CR2 |= 1;
	ADC1->CR2 &= ~(1<<11);							// Right alignment	
	ADC1->SMPR2 = 0;							// sampling time: 1.5 cycle 
	ADC1->SQR3 = 0x00000009;				// Select channel IN9
	ADC1->CR1 |= (1<<8);							// Enable scan mode
	ADC1->CR2 |= (7<<17);							// select SWSTART as external trigger
	// EOC interrupt enable
	ADC1->CR1 |= (1<<5); 
	__enable_irq();
	NVIC_EnableIRQ(ADC1_2_IRQn);
	// start calibration
	ADC1->CR2 |= (1<<2);

//  while( ADC1->CR2 & (1<<2));		// wait until calibration completed
	ADC1->CR2 |= (1<<0);							// enable ADC 
}
// #####################################################################################


int main(void){
	// enable clock for PORT B, AFIO, TIM4
	RCC->APB2ENR |= (1<<3)|(1<<0)|(3<<2);
	RCC->APB1ENR |= (1<<2);	
	
	
	// Set PA1~PA8 and PB0~PB3 as output
	GPIOA->CRL = 0x22222224;  // PA1~PA7 are output
	GPIOA->CRH = 0x44444442;  // PA8 is output
	GPIOB->CRH = 0x22224444;
	GPIOA->ODR = 0x000001E4;
	
	// Set PB5 as input, PB4 as input, PB8-13 output
	GPIOB->CRL = 0x4B444444; // PB0~7 
	GPIOB->ODR = 0;
	
	// Configure EXTI4
	AFIO->EXTICR[1] |= (1<<0);  // PB4 are selected as the source for EXTI
	EXTI->IMR |= (1<<4);        // enable EXTI4
	EXTI->RTSR |= (1<<4);   // detect rising edge
	
	__enable_irq();
	NVIC_EnableIRQ(EXTI4_IRQn);  // enable EXTI4 interrupt in NVIC
	Clock_Init();
	TIM4_Init();
	
	// Step 3: Create EXTI0
	AFIO->EXTICR[0] |= (1<<0);
	EXTI->IMR |= (1<<0);
	EXTI->FTSR |= (1<<0);

	NVIC_EnableIRQ(EXTI0_IRQn);
		
	

	// convert PB1 to analog input (reset state)
	GPIOB->CRL &= ~(1UL<<6); 
	ADC_Init();
	// Init systick timer
	SysTick_Config(9000);   // set systick timer,
	
	while(1){
		
		ADC1->CR2 |= (1<<0);
		
		
		TIM4->CCR1 = (adc_value/3.3)*20000;
		delay(100); // delay 500 ms
		display((int)(60*(count)/(0.01*390*2)));
		count = 0;

	}
}


void Clock_Init(void)
{
/*
	system clock 72MHz
	APB1 clock PCLK1 = 72/8 = 9MHz
*/
	// Set APB1 prescaler to 1/8 
//	RCC->CFGR |= (6<<8);
	// enable GPIOA, GPIOB, AFIO
	RCC->APB2ENR |= (1U<<0) | (1U<<2) | (1U<<3);
	// enable TIM4  and TIM2
	RCC->APB1ENR |=  (1U<<2) | (1U<<0);
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_ADC1EN;
}




void TIM4_Init(void) // TIM4 CH1 PWM output with PB6
{ 
	// enable clock to TIM4
	RCC->APB1ENR |= (1U<<2);
	// Set TIM4 prescaler to 1/18 -> Ftim4 = 1MHz
	TIM4->PSC = 18-1;     
	// Set ARR, CNT, CCR
	TIM4->ARR = 20000;
	// reset counter
	TIM4->CNT = 0;
	// config dutty cycle
	TIM4->CCR1 = 0; // dutty cycle 0%
	// Set PWM1 mode1 to CH1
	TIM4->CCMR1 |= (6U<<4);
	// Enable CH1
	TIM4->CCER |= (1U<<0);
	// Enable counter
	TIM4->CR1 |= (1U<<0);
}



void EXTI4_IRQHandler(void){
	EXTI->PR |= (1<<4);  // clear interrupt flag
	 count++;
}

void EXTI0_IRQHandler(void){
		EXTI->PR |= (1<<1);
		// Handle interupt
		count++;
}

void SysTick_Handler(void)
{
	 /* Increment counter necessary in Delay()*/
		msTicks++;
}

 
void delay(uint32_t dlyTicks)   // delay function
{
      uint32_t curTicks;
      curTicks = msTicks;
      while ((msTicks - curTicks) < dlyTicks) ;
}


void display(uint16_t number){
	disp[3] = number / 1000;
	disp[2] = number%1000/100;
	disp[1] = number%100/10;
	disp[0] = number%10;
	// display number
	for (int i=0; i<4; i++){
		GPIOB->ODR = (1<<(i+12));  // turn on led i
		GPIOA->ODR = digit[disp[i]]; 
		delay(5); // delay 5ms
		GPIOB->ODR = 0;
//		GPIOB->ODR &= ~(1<<(i+12));   // turn off led i	
	}
}