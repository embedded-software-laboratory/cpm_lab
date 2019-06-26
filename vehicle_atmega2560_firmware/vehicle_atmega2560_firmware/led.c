/*
 * led.c
 *
 * Created: 20.11.2018 09:50:00
 *  Author: maczijewski
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include "util.h"
#include "led.h"


// synchronous architecture
// LED period now handled by master
void led_set_state(spi_mosi_data_t* spi_mosi_data) {
	CLEAR_BIT(PORTC, 0);
	CLEAR_BIT(PORTC, 1);
	CLEAR_BIT(PORTC, 2);
	CLEAR_BIT(PORTC, 7);
	
	if(spi_mosi_data->LED1_enabled) {
		SET_BIT(PORTC, 0);
	}

	if(spi_mosi_data->LED2_enabled) {
		SET_BIT(PORTC, 1);
	}	
	
	if(spi_mosi_data->LED3_enabled) {
		SET_BIT(PORTC, 2);
	}

	if(spi_mosi_data->LED4_enabled) {
		SET_BIT(PORTC, 7);
	}
}


void led_setup() {
	SET_BIT(DDRC, 0);
	SET_BIT(DDRC, 1);
	SET_BIT(DDRC, 2);
	SET_BIT(DDRC, 7);
}