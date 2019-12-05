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

uint8_t identification_LED_period_ticks[26]  = { 1, 4, 7, 10, 13, 16, 7, 10, 13, 16, 19, 10, 13, 16, 19, 22, 13, 16, 19, 22, 25, 16, 19, 22, 25, 28 };
uint8_t identification_LED_enabled_ticks[26] = { 0, 2, 2,  2,  2,  2, 5,  5,  5,  5,  5,  8,  8,  8,  8,  8, 11, 11, 11, 11, 11, 14, 14, 14, 14, 14 };
uint8_t vehicle_id = 0;
uint32_t tick_count = 0;

void led_set_state(spi_mosi_data_t* spi_mosi_data) {
	CLEAR_BIT(PORTC, 0);
	CLEAR_BIT(PORTC, 1);
	CLEAR_BIT(PORTC, 2);
	
	vehicle_id = spi_mosi_data->vehicle_id;
	
	if(vehicle_id==0 && tick_count % 25 == 0)
	{
		SET_BIT(PORTC, 0);
		SET_BIT(PORTC, 1);
		SET_BIT(PORTC, 2);
	}	
	if (vehicle_id != 0)
	{
		SET_BIT(PORTC, 0);
		SET_BIT(PORTC, 1);
		SET_BIT(PORTC, 2);
	}
	
	
}


void led_setup() {
	SET_BIT(DDRC, 0);
	SET_BIT(DDRC, 1);
	SET_BIT(DDRC, 2);
	SET_BIT(DDRC, 7);
}

void toggle_led()
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