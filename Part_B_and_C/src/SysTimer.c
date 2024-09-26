/*
 * ECE 153B
 *
 * Name(s):
 * Section:
 * Project
 */

#include "SysTimer.h"
#include "motor.h"
#include "stm32l476xx.h"
static uint32_t volatile step;

void SysTick_Init(void) {
	// SysTick Control & Status Register
	SysTick->CTRL = 0; // Disable SysTick IRQ and SysTick Counter

	SysTick->LOAD = 9999;
	SysTick->VAL = 0;
	
	// Enables SysTick exception request
	// 1 = counting down to zero asserts the SysTick exception request
	// 0 = counting down to zero does not assert the SysTick exception request
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; 
	// Select clock source
	// If CLKSOURCE = 0, the external clock is used. The frequency of SysTick clock is the frequency of the AHB clock divided by 8.
	// If CLKSOURCE = 1, the processor clock is used.
	SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;		
	
}
uint32_t watchISR; 
uint32_t watchUARTISR;
uint32_t watchCSELR; 
void SysTick_Handler(void) {
	rotate(); 
	step++; //for delay
	
	
	watchISR = DMA1->ISR;
	watchCSELR = DMA1_CSELR->CSELR;
	watchUARTISR = USART1->ISR;
	//DMA1->IFCR |= ~0;
}

void delay(uint32_t T) {
	step = 0; // Hint: It may be helpful to keep track of what the current tick count is
	
	// [TODO] - Implement function that waits until a time specified by argument T
	while (step < T);
}
