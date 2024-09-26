/*
 * ECE 153B
 *
 * Name(s):
 * Section:
 * Project
 */

#include "stm32l476xx.h"
#include "motor.h"

static const uint32_t MASK = 0xFFFFFFFF;//TODO
static const uint32_t HalfStep[8] = {0x9,0x1,0x3,0x2,0x6,0x4,0xC,0x8};//TODO

static volatile int8_t dire = 0;
static volatile uint8_t step = 0;

void Motor_Init(void) {	
	//TODO
	// Enable GPIO Clocks
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	
	GPIOC->MODER &= ~(GPIO_MODER_MODE0|GPIO_MODER_MODE1|GPIO_MODER_MODE2|GPIO_MODER_MODE3);
	GPIOC->MODER |= GPIO_MODER_MODE0_0|GPIO_MODER_MODE1_0|GPIO_MODER_MODE2_0|GPIO_MODER_MODE3_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT0|GPIO_OTYPER_OT1|GPIO_OTYPER_OT2|GPIO_OTYPER_OT3);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR0|GPIO_PUPDR_PUPDR1|GPIO_PUPDR_PUPDR2|GPIO_PUPDR_PUPDR3);
	
	GPIOC->ODR &=  ~(GPIO_ODR_OD0|GPIO_ODR_OD1|GPIO_ODR_OD2|GPIO_ODR_OD3); 
	GPIOC->ODR |= HalfStep[step];//set position to 0 state. 
}

void rotate(void) {
	//TODO
	//printf("%d %d\n",dire,step);
	if (dire) { //if direction is nonzero (indicating CW)
		if (step==7){
			//printf("CW, switched to step 0\n");
			step=0;
		} else {
			step++;
		}
	} else {     //if direction is 0 (indicating CCW)
		if (step==0){
			//printf("CCW, switched to step 7\n");
			step=7;
		} else {
			step--;
		}
	}
	GPIOC->ODR &= ~(MASK);
	GPIOC->ODR |= HalfStep[step];
}

void setDire(int8_t direction) {
	//TODO
	//printf("received %d", direction);
	dire = direction;
}
	


