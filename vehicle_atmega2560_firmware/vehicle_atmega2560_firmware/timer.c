/*
 * timer.c
 *
 * Created: 08.01.2020 08:35:56
 *  Author: kloock
 */ 

#include "timer.h"

// timer 3 set to 20ms period
// use timer for LEDs and servo, as they have the same frequency-requirements
// timer/counter3 overflow interrupt
// used timer3 because convenient: could have used different timer
ISR(TIMER3_OVF_vect) {
	tick_counter++;
	toggle_led();
}


void timer_setup() {
	// Using timer 3
	
	// Fast PWM
	SET_BIT(TCCR3B, WGM33);
	SET_BIT(TCCR3B, WGM32);
	SET_BIT(TCCR3A, WGM31);
	SET_BIT(TCCR3A, WGM30);
	
	// Output on Pin 7 / OC3C / PE5
	SET_BIT(DDRE, 5);
	SET_BIT(TCCR3A, COM3C1);
	
	// prescaler /8 => 2 MHz
	SET_BIT(TCCR3B, CS31);
	
	// set frequency to 50Hz, unit: 0.5 usec
	OCR3A = 40000-1;
	
	// servo center position (1.5 msec)
	OCR3C = 3000;
	
	// TOP interrupt for tick timer
	// 50Hz i.e. 20ms
	SET_BIT(TIMSK3, TOIE3);
	SET_BIT(TIFR3, TOV3);
	
	// servo enable
	SET_BIT(DDRD, 7);
	SET_BIT(PORTD, 7);
}