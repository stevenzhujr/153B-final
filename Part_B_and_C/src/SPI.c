#include "SPI.h"
#include "SysTimer.h"

void SPI1_GPIO_Init(void) {
	// Enable the GPIO Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN; 
	// Set PA4, PB3, PB4, and PB5 to Alternative Functions, and configure their AFR to SPI1
	GPIOA->MODER &= ~(GPIO_MODER_MODE4);  
	GPIOA->MODER |= GPIO_MODER_MODE4_1; 
	GPIOA->AFR[0] |= 0x5 << 4*4; //set PA4 alternate function to AF5
	
	GPIOB->MODER &= ~(GPIO_MODER_MODE3|GPIO_MODER_MODE4|GPIO_MODER_MODE5);  
	GPIOB->MODER |= (GPIO_MODER_MODE3_1|GPIO_MODER_MODE4_1|GPIO_MODER_MODE5_1); 
	GPIOB->AFR[0] |= (0x5 << 3*4) | (0x5 << 4*4) | (0x5 << 5*4); //set PA4 alternate function to AF5
	
	// Set GPIO Pins to: Very High Output speed, Output Type Push-Pull, and No Pull-Up/Down
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED4; 
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT4;  
	GPIOA->PUPDR  &= ~GPIO_PUPDR_PUPD4; 
	
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED3|GPIO_OSPEEDR_OSPEED4|GPIO_OSPEEDR_OSPEED5;
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT3|GPIO_OTYPER_OT4|GPIO_OTYPER_OT5); 
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3|GPIO_PUPDR_PUPD4|GPIO_PUPDR_PUPD5); 
}


void SPI1_Init(void){
	// Enable SPI clock and Reset SPI
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; 
	RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
	// Disable SPI
	SPI1->CR1 &= ~SPI_CR1_SPE;
	// Configure for Full Duplex Communication
	SPI1->CR1 &= ~SPI_CR1_RXONLY;
	// Configure for 2-line Unidirectional Data Mode
	SPI1->CR1 &= ~SPI_CR1_BIDIMODE;
	// Disable Output in Bidirectional Mode
	SPI1->CR1 &= ~SPI_CR1_BIDIOE;
	// Set Frame Format: MSB First, 16-bit, Motorola Mode
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST;
	SPI1->CR2 |= SPI_CR2_DS;
	SPI1->CR2 &= ~SPI_CR2_FRF;
	// Configure Clock. Read DataSheet for required value
	SPI1->CR1 |= SPI_CR1_CPOL;
	SPI1->CR1 |= SPI_CR1_CPHA;
	// Set Baud Rate Prescaler to 16
	SPI1->CR1 &= ~SPI_CR1_BR;
	SPI1->CR1 |= 0x3 << 3;
	// Disable Hardware CRC Calculation
	SPI1->CR1 &= ~SPI_CR1_CRCEN;
	// Set as Master
	SPI1->CR1 |= SPI_CR1_MSTR;
	// Disable Software Slave Management
	SPI1->CR1 &= ~SPI_CR1_SSM;
	// Enable NSS Pulse Management
	SPI1->CR2 |= SPI_CR2_NSSP;
	// Enable Output
	SPI1->CR2 |= SPI_CR2_SSOE;
	// Set FIFO Reception Threshold to 1/2
	SPI1->CR2 &= ~SPI_CR2_FRXTH;
	// Enable SPI
	SPI1->CR1 |= SPI_CR1_SPE;
}

uint16_t SPI_Transfer_Data(uint16_t write_data){
	// Wait for TXE (Transmit buffer empty)
	while ((SPI1->SR&SPI_SR_TXE)==0);
	// Write data
	SPI1->DR = write_data;
	// Wait for not busy
	while ((SPI1->SR&SPI_SR_BSY)==1);
	// Wait for RXNE (Receive buffer not empty)
	while ((SPI1->SR&SPI_SR_RXNE)==0);
	// Read data
	return SPI1->DR&0xFF; //get lower 8 bits 
}