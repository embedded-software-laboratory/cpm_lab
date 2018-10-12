/*
 * motor.c
 *
 * Created: 20.09.2018 21:37:53
 *  Author: maczijewski
 */ 

#include "motor.h"
#include "util.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define MOTOR_DIRECTION_BRAKE 0
#define MOTOR_DIRECTION_FORWARD 1
#define MOTOR_DIRECTION_REVERSE 2

void motor_set_direction(uint8_t direction)
{
	if(direction & MOTOR_DIRECTION_REVERSE) {
		SET_BIT(PORTC, 3);
	}
	else {
		CLEAR_BIT(PORTC, 3);
	}
	
	if(direction & MOTOR_DIRECTION_FORWARD) {
		SET_BIT(PORTC, 5);
	} 
	else {
		CLEAR_BIT(PORTC, 5);
	}
}

void motor_set_duty(uint16_t duty) // values from 0 to 400
{
	if(duty > 400) {
		duty = 400;
	}
	OCR4B = duty;
}

void motor_setup()
{
	// PWM mode 9: PWM,Phase and Frequency Correct
	SET_BIT(TCCR4B, WGM43);
	SET_BIT(TCCR4A, WGM40);
	
	// Set PWM frequency to 20kHz
	OCR4A = 400;
	
	// Enable output on Pin 16 / OC4B / PH4
	SET_BIT(TCCR4A, COM4B1);
	SET_BIT(DDRH, 4);
	
	// no clock scaling, counter runs at 16MHz
	SET_BIT(TCCR4B, CS40);
	
	// Enable direction control
	SET_BIT(DDRC, 3);
	SET_BIT(DDRC, 5);
	motor_set_direction(MOTOR_DIRECTION_FORWARD);
}
