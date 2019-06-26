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
#include "led.h"
#include "watchdog.h"


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

    // timeout in 8K cycles (64ms), disable reset mode. Must be done in one operation
	// = not |= since want to clear WDE. Want to rewrite WDTCSR not change certain bits.
    WDTCSR = 1<<WDP2;

    // enable global interrupts
    sei();

    // enable watchdog interrupt only mode
    WDTCSR |= 1<<WDIE;
}


void watchdog_reset(){
	wdt_reset();
}


void safe_mode(spi_mosi_data_t* spi_mosi_data) {
	uint8_t tick = 0;
	while(1) {
		// SS LOW -> master active again
		if (~PINB & 0b00000001) { // reset safe mode
			safe_mode_flag = 1;
		}
		
		// uint8: 0-255	
		if (tick%127) {
			spi_mosi_data->LED1_enabled = 1;
			spi_mosi_data->LED2_enabled = 1;
			spi_mosi_data->LED3_enabled = 1;
			spi_mosi_data->LED4_enabled = 1;
		
			led_set_state(spi_mosi_data);
		}
		else {
			spi_mosi_data->LED1_enabled = 0;
			spi_mosi_data->LED2_enabled = 0;
			spi_mosi_data->LED3_enabled = 0;
			spi_mosi_data->LED4_enabled = 0;
			led_set_state(spi_mosi_data);		
		}
	
	tick++;	
	}
}


ISR(WDT_vect) {
	safe_mode_flag = 1;
}