/*
 * ECE 153B
 *
 * Name(s):
 * Section:
 * Project
 */
 
 
 
 
#include "DMA.h"
#include "SysTimer.h"

void DMA_Init_UARTx(DMA_Channel_TypeDef * tx, USART_TypeDef * uart) {
	//TODO
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	for (int i = 0; i < 2000; i++); //wait at least 20 us for DMA to finish setting up
	
	tx->CCR &= ~DMA_CCR_EN;
	tx->CCR &= ~DMA_CCR_MEM2MEM; //disable memory to memory mode
	tx->CCR |= 0x3 << 12; // set channel priority to high
	tx->CCR &= ~DMA_CCR_PSIZE; //set peripheral word size to 8 bit 
	tx->CCR &= ~DMA_CCR_MSIZE; //set memory word size to 8 bit
	tx->CCR &= ~DMA_CCR_PINC; //disable peripheral increment mode
	tx->CCR |= DMA_CCR_MINC; //enable memory increment mode
	tx->CCR &= ~DMA_CCR_CIRC; //disable circular mode
	tx->CCR |= DMA_CCR_DIR; //Set data transfer direction from memory to peripheral //write mode
	
	//need to set tx->CMAR, but we do this elsewhere
	//tx->CPAR = (uint32_t) uart->TDR; //set peripheral buffer to the UART TDR
	
	//Interrupts not needed, using USART interrupts instead
	tx->CCR &= ~DMA_CCR_HTIE;
	tx->CCR &= ~DMA_CCR_TEIE;
	tx->CCR |= DMA_CCR_TCIE; //unless?
	
	//Clear all interrupt flags:
	
	//enable DMA1_Channels for appropriate USART ports
	if (uart == USART1) {
		DMA1_CSELR->CSELR	|= 0x2 << 12; //map channel 4 to USART1TX
	} else if (uart==USART2) {
		DMA1_CSELR->CSELR	|= 0x2 << 24; //map channel 7 to USART2TX
	}
	
	if (tx == DMA1_Channel4) {
		DMA1->IFCR |= 0x15 << 12; //clear any flags
		NVIC_SetPriority(DMA1_Channel4_IRQn,3); //figure this out
		NVIC_EnableIRQ(DMA1_Channel4_IRQn);
		NVIC_ClearPendingIRQ(DMA1_Channel4_IRQn);
	} else if (tx==DMA1_Channel7) {
		DMA1->IFCR |= 0x15 << 24; //clear any flags
		NVIC_SetPriority(DMA1_Channel7_IRQn,3); //figure this out
		NVIC_EnableIRQ(DMA1_Channel7_IRQn);
		NVIC_ClearPendingIRQ(DMA1_Channel7_IRQn);
	}
	
	
}




