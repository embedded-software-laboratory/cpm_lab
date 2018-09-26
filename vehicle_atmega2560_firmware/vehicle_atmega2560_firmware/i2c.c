/*
 * i2c.c
 *
 * Created: 26.09.2018 17:59:03
 *  Author: maczijewski
 */ 


#include <avr/interrupt.h>
#include <avr/io.h>
#include "util.h"

#define I2C_STATUS (TWSR & 0b11111000)

ISR(TWI_vect) {
	
}


void i2c_setup() {
	SET_BIT(TWCR, TWEN); // enable I2C
	SET_BIT(PORTD, PD1); // pullup SDA
	SET_BIT(PORTD, PD0); // pullup SCL
	TWBR = 72; // SCL freq 100kHz
	SET_BIT(TWCR, TWIE); // enable I2C interrupt
	SET_BIT(TWCR, TWEA); // enable Acknowledge Bit
}