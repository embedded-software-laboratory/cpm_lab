/*
 * watchdog.c
 *
 * Created: 6/21/2019 13:35:14
 *  Author: cfrauzem
 */ 


#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdint.h>
#include "servo_timer.h"
#include "led.h"
#include "watchdog.h"

volatile uint8_t safe_mode_flag = 0;


void watchdog_disable() {
	wdt_reset();
	
	// disable interrupts so we do not get interrupted while doing timed sequence
	cli();
	
	// clear WDRF in MCUSR: can not clear WDE if WDRF is set
	MCUSR &= ~(1<<WDRF);
	
	// write logical one to WDCE and WDE
	// first step of timed sequence, we have 4 cycles after this to make changes to WDE and WD Timeout
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	
	// turn off WDT
	WDTCSR = 0x00;
	
	// enable global interrupts
	sei();
}


void watchdog_enable() {
	wdt_reset();
	
	// disable interrupts so we do not get interrupted while doing timed sequence
	cli();
		
    // Just to be safe since we can not clear WDE if WDRF is set
    MCUSR &= ~(1<<WDRF);

	// Start timed sequence
    // First step of timed sequence, we have 4 cycles after this to make changes to WDE and WD timeout
    WDTCSR |= (1<<WDCE) | (1<<WDE);

    // timeout in 32K cycles (0.25s), disable reset mode. Must be done in one operation
	// = not |= since want to also clear WDE -> need rewrite WDTCSR not change certain bits.
    WDTCSR = 1<<WDP2; // 250ms
	//WDTCSR = (1<<WDP3) | (1<<WDP0); // 8sec

    // enable watchdog interrupt only mode
    WDTCSR |= 1<<WDIE;
    
    // enable global interrupts
    sei();
}


void watchdog_reset() {
	wdt_reset();
	safe_mode_flag = 0;
}

ISR(WDT_vect) {
	safe_mode_flag = 1;
}