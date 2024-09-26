/*
 * ECE 153B
 *
 * Name(s):
 * Section:
 * Project
 */

#include "LED.h"

void LED_Init(void) {
	// Enable GPIO Clocks
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; 
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN; 
	
	GPIOA->MODER &= ~(GPIO_MODER_MODE5); //configure PA5 as output
	GPIOA->MODER |= GPIO_MODER_MODE5_0;
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5); //configure output mode as push-pull
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR5); //Configure no pullup pull-down 
	
	GPIOC->MODER &= ~(GPIO_MODER_MODE13); //configure PC13 as input
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR13); //configure no pull-up no pull-down 
	
	// Initialize Green LED
	GPIOA->ODR |= GPIO_ODR_OD5; 
}

void LED_Off(void) {
	GPIOA->ODR &= GPIO_ODR_OD5; 
}

void LED_On(void) {
	GPIOA->ODR |= GPIO_ODR_OD5;
}

void LED_Toggle(void){
	GPIOA->ODR ^= GPIO_ODR_OD5; 
}
