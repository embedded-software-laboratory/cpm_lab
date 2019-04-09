/*
 * servo_timer.c
 *
 * Created: 24.09.2018 16:26:48
 *  Author: maczijewski
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include "util.h"
#include "servo_timer.h"


static volatile uint32_t tick_counter = 0;
static volatile uint8_t tick_flag = 0;


void set_servo_pwm(uint16_t pwm) {	
	if(pwm > 4000) pwm = 4000;
	else if (pwm < 2000) pwm = 2000;
	OCR3C = pwm;
}

uint32_t get_tick() { return tick_counter; }

void tick_wait() {
	tick_flag = 1;
	while(tick_flag);
}

ISR(TIMER3_OVF_vect) {
	tick_counter++;
	tick_flag = 0;
}

void servo_timer_setup() {
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
	SET_BIT(TIMSK3, TOIE3);
	SET_BIT(TIFR3, TOV3);
}