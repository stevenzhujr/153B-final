#include "SPI.h"
#include "SysTimer.h"
#include "accelerometer.h"

void accWrite(uint8_t addr, uint8_t val){
	// TODO access SPI_Transfer_Data
	uint16_t commandByte = ((addr&0b111111) << 8) | val; //mask addr to 6 bits, val is uint8_t so is fine 
	SPI_Transfer_Data(commandByte); //discard returned value
}

//CHECK THIS FUNCTION IF RETURNED DATA LOOKS LIKE 0, may need to do bitshifting to return correct portion of DR
uint8_t accRead(uint8_t addr){
	// access SPI_Transfer_Data
	uint16_t commandByte = (0b10 << 14) | (addr << 8); //set read bit to high, MB bit to 0, lower 8 bits don't matter for reading
	return SPI_Transfer_Data(commandByte); // TODO - truncates 8 MSB of return value, but according to timing diagram only the 8 LSB matters for data
}

void initAcc(void){
	// set full range mode
	accWrite(0x31, 0b00001000); //SELF_TEST SPI = 4wiremode INT_INVERT=activelow interrupts 0 FULL_RES=enabled Justify=MSB on left Range = +-16 g
	accWrite(0x2C, 0b00001010); //0 0 0 LOW_POWER=disabled(normal operation) Rate = 100 Hz output data rate, Bandwidth = 50z, 
	// enable measurement
	accWrite(0x2D, 0b00111011); //0 0 Link=don'tknow AUTO_SLEEP=enabled Measure=enabled Sleep=disabled Wakeup=1 Hz
}

void readValues(double* x, double* y, double* z){
	// TODO
	uint16_t xReg, yReg, zReg; 
	/*
	0x32 50 DATAX0 R 00000000 X-Axis Data 0
	0x33 51 DATAX1 R 00000000 X-Axis Data 1
0x34 52 DATAY0 R 00000000 Y-Axis Data 0
0x35 53 DATAY1 R 00000000 Y-Axis Data 1
0x36 54 DATAZ0 R 00000000 Z-Axis Data 0
0x37 55 DATAZ1 */
	// find scaler from data sheet : All g-ranges, full resolution 3.5, 3.9, 4.3 mg/LSB
	// read values into x,y,z using accRead
	xReg = accRead(0x33)<<8|accRead(0x32); 
	yReg = accRead(0x35)<<8|accRead(0x34);
	zReg = accRead(0x37)<<8|accRead(0x36);
	*x = (int16_t)xReg*4E-3;
	*y = (int16_t)yReg*4E-3; 
	*z = (int16_t)zReg*4E-3;
	return;
}