/**
 * \file servo_timer.c
 *
 * \author maczijewski, cfrauzem
 * \date Created: 24.09.2018 16:26:48, Modified: 05.31.2019 11:32:19
 * 
 * \ingroup low_level_controller
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include "util.h"
#include "servo_timer.h"
#include "led.h"

/**
 * \brief Defines after how many steps in which the PWM signal for the servo did not change
 * 		  the servo gets disabled. Avoids unpleasant noises in high frequencies.
 * \ingroup low_level_controller
 */
#define SERVO_CENTER_COMMAND_COUNT_THRESHOLD 100

/**
 * \brief Counts ticks of this low_level_controller in 20ms periods. This variable is also
 * 		  used by the LEDs to control their flashing patterns.
 * \ingroup low_level_controller
 */
static volatile uint32_t tick_counter = 0;

/**
 * \brief Counts the number of consecutive commands in which the wanted servo PWM stays the same.
 *   	  As soon as it reaches \link SERVO_CENTER_COMMAND_COUNT_THRESHOLD \endlink the servo
 * 		  gets disabled.
 * \ingroup low_level_controller
 */
static uint8_t consecutive_servo_center_command_count = 0;

/* 
 * servo enable on pin PD7
 * servo pwm range:
 * 1800: negative limit which the servo can recognize
 * 3000: middle
 * 4200: positive limit which the servo can recognize
 */
void set_servo_pwm(uint16_t pwm) {	
	// cap command above servo command positive limit
	if(pwm > 4200) {
		pwm = 4200;
	}
	// cap command below servo command negative limit
	else if (pwm < 1800) {
		pwm = 1800;
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


/**
 * \brief Timer 3 set to 20ms period. Used for PWM of servo but also for the
 * 		  flashing behaviour of the LEDs since they have the same frequency-requirements.
 * 
 * timer/counter3 overflow interrupt
 * 
 * used timer3 because convenient: could have used different timer
 * \ingroup low_level_controller
 */
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
