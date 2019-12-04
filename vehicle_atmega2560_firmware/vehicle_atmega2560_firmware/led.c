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


void led_set_state(spi_mosi_data_t* spi_mosi_data) {
	CLEAR_BIT(PORTC, 0);
	CLEAR_BIT(PORTC, 1);
	CLEAR_BIT(PORTC, 2);
	//CLEAR_BIT(PORTC, 7);
	
	if(spi_mosi_data->LED1_enabled) {
		SET_BIT(PORTC, 0);
	}

	if(spi_mosi_data->LED2_enabled) {
		SET_BIT(PORTC, 1);
	}	
	
	if(spi_mosi_data->LED3_enabled) {
		SET_BIT(PORTC, 2);
	}

	//if(spi_mosi_data->LED4_enabled) {
	//	SET_BIT(PORTC, 7);
	//}
}


void led_setup() {
	SET_BIT(DDRC, 0);
	SET_BIT(DDRC, 1);
	SET_BIT(DDRC, 2);
	SET_BIT(DDRC, 7);

	// Using timer 5
	
	// Fast PWM
	SET_BIT(TCCR5B, WGM53);
	SET_BIT(TCCR5B, WGM52);
	SET_BIT(TCCR5A, WGM51);
	SET_BIT(TCCR5A, WGM50);
	
	// Output on Pin 7 / OC5C / PE5
	SET_BIT(DDRE, 5);
	SET_BIT(TCCR5A, COM5C1);
	
	// prescaler /8 => 2 MHz
	SET_BIT(TCCR5B, CS51);
	
	// set frequency to 50Hz, unit: 0.5 usec
	OCR5A = 40000-1;
	
	// servo center position (1.5 msec)
	OCR5C = 3000;
	
	// TOP interrupt for tick timer
	// 50Hz i.e. 20ms
	SET_BIT(TIMSK5, TOIE5);
	SET_BIT(TIFR5, TOV5);
}

ISR(TIMER5_CAPT_vec)
{
	tick_count++;
	if(tick_count % identification_LED_period_ticks[vehicle_id] < identification_LED_enabled_ticks[vehicle_id])
        {
            SET_BIT(PORTC, 7);
        }
	else
		{
			CLEAR_BIT(PORTC, 7);
		}
}