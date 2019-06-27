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

// safe mode tick
static volatile uint32_t tick = 0;


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
	
    // enable global interrupts
    sei();

    // enable watchdog interrupt only mode
    WDTCSR |= 1<<WDIE;
}


void watchdog_reset(){
	wdt_reset();
}


void safe_mode(spi_mosi_data_t* spi_mosi_data) {
	
	while(1) {
		uint32_t tick = get_tick();
			
		// SS LOW -> master active again
		if (~PINB & 0b00000001) { 
			safe_mode_flag = 0; // reset safe mode
			watchdog_reset(); // reset watchdog
			return;
		}
		
		// 50 ticks = 1sec 
		if (tick%30>15) {
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
	}
}


ISR(WDT_vect) {
	safe_mode_flag = 1;
}