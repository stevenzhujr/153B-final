/*
 * ECE 153B
 *
 * Name(s):
 * Section:
 * Project
 */

#include "stm32l476xx.h"
#include "SysClock.h"
#include "SysTimer.h"
#include "LED.h"
#include "DMA.h"
#include "UART.h"
#include "motor.h"
#include "SPI.h"
#include "I2C.h"
#include "accelerometer.h"
#include <stdio.h>
#include <string.h>

static char buffer[IO_SIZE];

void UART_onInput(char* inputs, uint32_t size) {
	//TODO
	if (strcmp(inputs,"cw\n")==0 
	|| strcmp(inputs,"1\n")==0
		|| strcmp(inputs,"CW\n")==0
	) {
		setDire(1);
		sprintf(buffer,"dir is cw\n");
		UART_print(buffer);
	} else if (strcmp(inputs,"ccw\n")==0
	|| strcmp(inputs,"0\n")==0
		|| strcmp(inputs,"CCW\n")==0) {
		setDire(0);
		sprintf(buffer,"dir is ccw\n");
		UART_print(buffer);
	} 
}

int main(void) {
	// Switch System Clock = 80 MHz
	System_Clock_Init(); 
	Motor_Init();
	SysTick_Init();
	USART_Init(USART1); //UART2_Init(); //implements non-blocking DMA with 
	LED_Init();	
	SPI1_GPIO_Init();
	SPI1_Init();
	initAcc();
	I2C_GPIO_Init();
	I2C_Initialization();
	
	sprintf(buffer, "Program Starts.\r\n");
	UART_print(buffer);
	char ch;
	
	while(1) {
		//TODO
		LED_Toggle();
		
		//UART_print(buffer);
		delay(1000);
	}
}


