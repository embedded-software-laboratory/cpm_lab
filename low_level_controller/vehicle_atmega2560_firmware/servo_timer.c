// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

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

static volatile uint32_t tick_counter = 0;

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


// timer 3 set to 20ms period
// timer/counter3 overflow interrupt
// used timer3 because convenient: could have used different timer
ISR(TIMER3_OVF_vect) {
	tick_counter++;
	// use the same timer for LEDs, as it has the same frequency-requirements 
	led_toggle();
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
	// 50Hz i.e. 20ms
	SET_BIT(TIMSK3, TOIE3);
	SET_BIT(TIFR3, TOV3);
	
	// servo enable
	SET_BIT(DDRD, 7);
	SET_BIT(PORTD, 7);
}