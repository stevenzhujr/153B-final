/*
 * ECE 153B
 *
 * Name(s):
 * Section:
 * Project
 */


#include "UART.h"
#include "DMA.h"
#include <string.h>
#include <stdio.h>

static volatile DMA_Channel_TypeDef * tx;
static volatile char inputs[IO_SIZE];
static volatile uint8_t data_t_0[IO_SIZE];
static volatile uint8_t data_t_1[IO_SIZE];
static volatile uint8_t input_size = 0;
static volatile uint8_t pending_size = 0;
static volatile uint8_t * active = data_t_0;
static volatile uint8_t * pending = data_t_1;

#define SEL_0 1
#define BUF_0_EMPTY 2
#define BUF_1_EMPTY 4
#define BUF_0_PENDING 8
#define BUF_1_PENDING 16

void transfer_data(char ch);
void on_complete_transfer(void);

void UART1_Init(void) {
	// [TODO]
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; //enable peripheral clock for USART
	RCC->CCIPR |= RCC_CCIPR_USART1SEL_0; //configure USART to use system clock
}

void UART2_Init(void) {
	// [TODO]
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; //enable peripheral clock for USART
	RCC->CCIPR |= RCC_CCIPR_USART2SEL_0; //configure USART to use system clock
}

void UART1_GPIO_Init(void) {
	// [TODO]
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7); //clear entire register
	GPIOB->MODER |= GPIO_MODER_MODE6_1; //set to AF mode
	GPIOB->MODER |= GPIO_MODER_MODE7_1;
	
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6; //set to very high output speed
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7;
	
	GPIOB->AFR[0] |= 0x7 << 24; //set to alternate function 7 (USART1_TX)
	GPIOB->AFR[0] |= 0x7 << 28; //set to to AF7 (USART1_RX)
	//all pins set to push-pull in reset state;
	
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6|GPIO_PUPDR_PUPD7);
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD6_0; //set to pull-up resistor only
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD7_0; 
}

void UART2_GPIO_Init(void) {
	// [TODO]
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3); //clear entire register
	GPIOA->MODER |= GPIO_MODER_MODE2_1; //set to AF mode
	GPIOA->MODER |= GPIO_MODER_MODE3_1;
	
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2; //set to very high output speed
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;
	
	GPIOA->AFR[0] |= 0x7 << 8; //set to alternate function 7 (USART_TX)
	GPIOA->AFR[0] |= 0x7 << 12; //set to to AF7 (USART_RX)
	//all pins set to push-pull in reset state;
	
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD2|GPIO_PUPDR_PUPD3);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD2_0; //set to pull-up resistor only
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD3_0; 
}

void USART_Init(USART_TypeDef* USARTx) {
	// Disable USART before configuring settings
	USARTx->CR1 &= ~USART_CR1_UE;
	
	//From Memory table, let's use DMA1, then USART1_TX is Channel 4, USART2_TX is channel 7
	if (USARTx==USART1) {
		tx = DMA1_Channel4; 
		UART1_Init();
		UART1_GPIO_Init();
	} else if (USARTx==USART2) {
		tx = DMA1_Channel7;
		UART2_Init();
		UART2_GPIO_Init();
	}DMA_Init_UARTx(tx,USARTx);
	
	//setting src and dest addresses
	tx->CMAR = (volatile uint32_t) active;
	tx->CPAR = (uint32_t)(&USARTx->TDR);
	
	// Set Communication Parameters
	USARTx->CR1 &= ~(USART_CR1_M);     // 00 -> 8 Data Bits
	USARTx->CR1 &= ~(USART_CR1_OVER8); // 0 -> Oversampling by 16
	USARTx->CR2 &= ~(USART_CR2_STOP);  // 00 -> 1 Stop Bit
	
	// Set Baud Rate
	// f_CLK = 80 MHz, Baud Rate = 9600 = 80 MHz / DIV -> DIV = 8333 = 0x208D
	USARTx->BRR &= 0;
	USARTx->BRR = 0x208D;
	
	//Enable DMA for the transmitter
	USARTx->CR3 |= USART_CR3_DMAT;
	// Enable Transmitter/Receiver
	USARTx->CR1 |= USART_CR1_TE | USART_CR1_RE;
	
	//Don't know what this is for, but it will make it work
	USARTx->RTOR = 20 << 24; //let the block length be 20 characters, including CRC   
	
	//enable interrupts for transmission complete and receiving data
	USARTx->ICR |= USART_ICR_TCCF; USARTx->TDR = 0;
	//while (USART_ISR_TC & USARTx->ISR);
	USARTx->CR1 |=  USART_CR1_RXNEIE; 
	
	
	if (USARTx == USART1) {
		NVIC_SetPriority(USART1_IRQn,0); //figure this out
		NVIC_EnableIRQ(USART1_IRQn);
		NVIC_ClearPendingIRQ(USART1_IRQn);
	} else if (USARTx==USART2) {
		NVIC_SetPriority(USART2_IRQn,0); //figure this out
		NVIC_EnableIRQ(USART2_IRQn);
		NVIC_ClearPendingIRQ(USART2_IRQn);
	}
	
	
	
	
	USARTx->ICR |= USART_ICR_TCCF; //clear the transmission complete flag
	
	// Enable USART
	USARTx->CR1 |= USART_CR1_UE;
	//Enable DMA
}

/**
 * This function accepts a string that should be sent through UART
*/

//DMA_TypeDef * watchDMA;
void UART_print(char* data) {
	//TODO
	//watchDMA = (DMA_TypeDef *)DMA1_BASE;
	//Transfer char array to buffer
	//won't do, already have data??

	//Check DMA status. If DMA is ready, send data
	if (tx->CNDTR==0) {
		tx->CCR&=~DMA_CCR_EN;
		strcpy(active,data);
		
		tx->CNDTR = strlen(data);// watchCNDTR=tx->CNDTR;
	
		if (tx == DMA1_Channel4) {
			//DMA1->IFCR |= 0x15 << 12; //clear any flags
		} else if (tx == DMA1_Channel7) {
			//DMA1->IFCR |= 0x15 << 24; //clear any flags
		}
		tx->CCR|=DMA_CCR_EN;
		
	} else { 
	//If DMA is not ready, put the data aside			
		sprintf(pending+pending_size, data);
		pending_size += strlen(data);
	}
}

/**
 * This function should be invoked when a character is accepted through UART
*/
void transfer_data(char ch) {
	//TODO
	// Append character to input buffer.
	// If the character is end-of-line, invoke UART_onInput
	inputs[input_size++]=ch;
	if (ch == '\n') {
		UART_onInput(inputs, input_size);
		input_size=0; inputs[0] = 0;
	}
}

/**
 * This function should be invoked when DMA transaction is completed
*/
uint8_t * temp;
void on_complete_transfer(void) {
	tx->CCR &= ~DMA_CCR_EN;
	//TODO
	// If there are pending data to send, switch active and pending buffer, and send data
	
	if (pending_size>0) {
		temp = active;
		active = pending;
		pending = temp;
		
		tx->CNDTR = pending_size;
		tx->CCR |= DMA_CCR_EN;
		pending_size = 0;
		//active register has contents of pending register, pending register is now "empty"
	}
	//__set_BASEPRI(0);
}

int watchUS1IRQ = 0;
void USART1_IRQHandler(void){
	//TODO
	// When receive a character, invoke transfer_data
	
	NVIC_ClearPendingIRQ(USART1_IRQn);
	watchUS1IRQ = 1;
	if (USART1->ISR & USART_ISR_ORE) {
		USART1->ICR |= USART_ICR_ORECF;
		transfer_data(USART1->RDR&0xFF);
		
	}
	
	if (USART1->ISR & USART_ISR_RXNE) {
		transfer_data(USART1->RDR&0xFF);
	}
	// When complete sending data, invoke on_complete_transfer
	else if (USART1->ISR & USART_ISR_TC) {
		USART1->ICR |=USART_ICR_TCCF;
		on_complete_transfer();
	}
}

int watchUS2IRQ = 0;
void USART2_IRQHandler(void){
	
	NVIC_ClearPendingIRQ(USART2_IRQn);
	//TODO
	// When receive a character, invoke transfer_data
	watchUS2IRQ = 1;
	if (USART2->ISR & USART_ISR_RXNE) {
		transfer_data(USART2->RDR&0xFF); //reading RDR clears RXNE automatically 
	}
	// When complete sending data, invoke on_complete_transfer ???? use DMA transfer???
	else if (USART2->ISR & USART_ISR_TC) {
		USART2->ICR |=USART_ICR_TCCF;
		on_complete_transfer(); 
	} 
}

//perhaps all this is not necessary
int watchC4IRQHandling = 0;
int watchC7IRQHandling = 0;
void DMA1_Channel4_IRQHandler(void){ 
	// TODO
	watchC4IRQHandling = 1;
	//watchCNDTR = DMA1_Channel4->CNDTR;
	NVIC_ClearPendingIRQ(DMA1_Channel4_IRQn);
	//printf("%s","handling interrupt\n");
	if (DMA1->ISR & DMA_ISR_TCIF4) {
		//while ((USART1->ISR & USART_ISR_TC)==0); //wait until the UART is ready
		DMA1->IFCR |= DMA_IFCR_CTCIF4; //clear transfer complete flag
		
		on_complete_transfer(); 
	}
	DMA1->IFCR |= DMA_IFCR_CGIF4; //clear global interrupt flag
}
void DMA1_Channel7_IRQHandler(void){ 
	// TODO
	watchC7IRQHandling = 1;
	NVIC_ClearPendingIRQ(DMA1_Channel7_IRQn);
	if (DMA1->ISR & DMA_ISR_TCIF7) {
		//while ((USART2->ISR & USART_ISR_TC)==0); //wait until the UART is ready
		DMA1->IFCR |= DMA_IFCR_CTCIF7; //clear transfer complete flag
		
		on_complete_transfer(); 
	}
	DMA1->IFCR |= DMA_IFCR_CGIF7; //clear global interrupt flag
}
