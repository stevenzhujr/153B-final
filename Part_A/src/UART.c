#include "UART.h"

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
	
	GPIOB->AFR[0] |= 0x7 << 24; //set to alternate function 7 (USART_TX)
	GPIOB->AFR[0] |= 0x7 << 28; //set to to AF7 (USART_RX)
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
	
	// Set Communication Parameters
	USARTx->CR1 &= ~(USART_CR1_M);     // 00 -> 8 Data Bits
	USARTx->CR1 &= ~(USART_CR1_OVER8); // 0 -> Oversampling by 16
	USARTx->CR2 &= ~(USART_CR2_STOP);  // 00 -> 1 Stop Bit
	
	// Set Baud Rate
	// f_CLK = 80 MHz, Baud Rate = 9600 = 80 MHz / DIV -> DIV = 8333 = 0x208D
	USARTx->BRR = 0x208D;
	
	// Enable Transmitter/Receiver
	USARTx->CR1 |= USART_CR1_TE | USART_CR1_RE;
	
	// Enable USART
	USARTx->CR1 |= USART_CR1_UE;
}

uint8_t USART_Read (USART_TypeDef * USARTx) {
	// SR_RXNE (Read data register not empty) bit is set by hardware
	while (!(USARTx->ISR & USART_ISR_RXNE));  // Wait until RXNE (RX not empty) bit is set
	// USART resets the RXNE flag automatically after reading DR
	return ((uint8_t)(USARTx->RDR & 0xFF));
	// Reading USART_DR automatically clears the RXNE flag 
}

void USART_Write(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes) {
	int i;
	// TXE is cleared by a write to the USART_DR register.
	// TXE is set by hardware when the content of the TDR 
	// register has been transferred into the shift register.
	for (i = 0; i < nBytes; i++) {
		while (!(USARTx->ISR & USART_ISR_TXE));   	// wait until TXE (TX empty) bit is set
		// Writing USART_DR automatically clears the TXE flag 	
		USARTx->TDR = buffer[i] & 0xFF;
		USART_Delay(5000); //300 for serial communication, 1000 for bluetooth
	}
	while (!(USARTx->ISR & USART_ISR_TC));   		  // wait until TC bit is set
	USARTx->ISR &= ~USART_ISR_TC;
}   

void USART_Delay(uint32_t us) {
	uint32_t time = 100*us/7;    
	while(--time);   
}
