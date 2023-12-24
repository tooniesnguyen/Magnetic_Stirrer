#include "stm32f10x.h"                  // Device header
#include <stdint.h>

void delay(uint32_t dlyTicks);
/* Counts 1ms timeTicks */
volatile uint32_t msTicks = 0;
void SysTick_Handler(void);
void display(uint16_t number);

uint16_t disp[4] = {0,0,0,0};
uint16_t digit[10] = {0x0E07,0x0F3F,0x0E4B,0x0E1B,0x0F33,0x0E93,0x0E83,0x0E3F,0x0E03,0x0E13};
                  //     0      1      2      3      4     5      6      7      8      9

// ###################################### ADC ############################################
//uint16_t adc_value;


//void ADC1_2_IRQHandler (void){
//	// read ADC value and clear EOC flag
//	adc_value = ADC1->DR;

//}
//void clockInit(){
//	// enable GPIOA, AFIO, ADC clock
//	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;
//	 
//}
//void ADC_Init(){
//	
//	// configure the ADC
//	ADC1->CR2 &= ~(1<<1);							// Single mode
//	ADC1->CR2 &= ~(1<<11);							// Right alignment	
//	ADC1->SMPR2 = 0;							// sampling time: 1.5 cycle 
//	ADC1->SQR3 = 0x000000A1;				// Select channel IN1 and IN5
//	ADC1->CR1 |= (1<<8);							// Enable scan mode
//	ADC1->CR2 |= (7<<17);							// select SWSTART as external trigger
//	// EOC interrupt enable
//	ADC1->CR1 |= (1<<5); 
//	__enable_irq();
//	NVIC_EnableIRQ(ADC1_2_IRQn);
//	// start calibration
//	ADC1->CR2 |= (1<<2);
////	while( ADC1->CR2 & (1<<2));		// wait until calibration completed
//	ADC1->CR2 |= (1<<0);							// enable ADC 
//}
// #####################################################################################


int main(void){
//	clockInit();
//	// convert PA10 to analog input (reset state)
//	GPIOA->CRH &= ~(1UL<<10); 
//	ADC_Init();
//	
	
	
	
	

	// Enable clock for Port A,B
	RCC->APB2ENR |= (3<<2); 
	
	// Set PA1~PA8 and PB0~PB3 as output
	GPIOA->CRL = 0x22222224;  // PA1~PA7 are output
	GPIOA->CRH = 0x44444442;  // PA8 is output
	GPIOB->CRH = 0x22224444;  // PB0~PB3 are output
	
	// Display
	GPIOA->ODR = 0x000001E4;
	GPIOB->ODR = 0x000F;   // turn off leds
	
	SysTick_Config(9000);   // set systick timer,
	
	while(1){
		// sweep 7-segment led
		for (int n=0; n<9999; n++){
			display(n);
			delay(100);
		}
	}
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