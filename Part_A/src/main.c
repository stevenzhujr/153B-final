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
#include "UART.h"
#include "motor.h"
#include <stdio.h>

void Init_USARTx(int x) {
    if(x == 1) {
        UART1_Init();
        UART1_GPIO_Init();
        USART_Init(USART1);
    } else if(x == 2) {
        UART2_Init();
        UART2_GPIO_Init();
        USART_Init(USART2);
    } else {
        // Do nothing...
    }
}
int main(void) {
	char ch;
	// Switch System Clock = 80 MHz
	System_Clock_Init(); 
	Motor_Init();
	SysTick_Init();
	
	Init_USARTx(1);
	
	printf("Program Starts.\r\n");
	
	//tests();
	while(1) {
		//TODO
		
		//Blocking UART Operation
		scanf("%c", &ch);
		//printf("%s\n",ch);
		if (ch == '1'  || (ch) == '0') {
			if (ch == '0') {
				setDire(0);
			} else {
				setDire(1);
			}
		}	
	
	}
}


