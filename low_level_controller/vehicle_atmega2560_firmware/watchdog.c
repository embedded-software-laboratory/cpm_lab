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

/**
 * \file watchdog.c
 *
 * \author cfrauzem
 * \date Created: 6/21/2019 13:35:14
 * 
 * \ingroup low_level_controller
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