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
#include "led.h"


#define SERVO_CENTER_COMMAND_COUNT_THRESHOLD 100

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
		CLEAR_BIT(TCCR3A, COM3C1); // disable pwm signal -> servo goes into fail safe limp mode
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
		SET_BIT(TCCR3A, COM3C1); // enable pwm signal
	}
}


uint32_t get_tick() { 
	return tick_counter; 
}


