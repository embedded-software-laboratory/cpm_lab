/*
 * servo_timer.c
 *
 * Created: 24.09.2018 16:26:48
 *  Author: maczijewski
 * Modified: 05.31.2019 11:32:19
 *  Author: cfrauzem
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include "util.h"
#include "servo_timer.h"


#define SERVO_CENTER_COMMAND_COUNT_THRESHOLD 100

static volatile uint32_t tick_counter = 0;
static volatile uint8_t tick_flag = 0;

static uint8_t consecutive_servo_center_command_count = 0;

/* 
 * servo enable on pin PD7
 * servo pwm range:
 * 2000: negative limit
 * 3000: middle
 * 4000: positive limit
 */
void set_servo_pwm(uint16_t pwm) {	
	// cap command above servo command positive limit
	if(pwm > 4000) {
		pwm = 4000;
	}
	// cap command below servo command negative limit
	else if (pwm < 2000) {
		pwm = 2000;
	}
	
	OCR3C = pwm;
	
	if (consecutive_servo_center_command_count > SERVO_CENTER_COMMAND_COUNT_THRESHOLD) {
		// disable servo
		CLEAR_BIT(PORTD, 7);
	}
	// need to consider overflow of 8-bit variable:
	// only increment counter while less than threshold
	else {
			if (pwm == 3000) {
				consecutive_servo_center_command_count++;
			}
	}
	
	// this is not the center/default command
	// reset the counter, re-enable the servo
	if (pwm != 3000) {
		consecutive_servo_center_command_count = 0;
		SET_BIT(PORTD, 7);
	}
}


//uint32_t get_tick() { 
	//return tick_counter; 
//}


//void tick_wait() {
	//tick_flag = 1;
	//while(tick_flag);
//}


// timer 3 set to 20ms period
// timer/counter3 overflow interrupt
// used timer3 becuase convenient could have used different timer
//ISR(TIMER3_OVF_vect) {
	//tick_counter++;
	//tick_flag = 0;
//}


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
	
	// servo enable
	SET_BIT(DDRD, 7);
	SET_BIT(PORTD, 7);
}