/*
 * led.c
 *
 * Created: 20.11.2018 09:50:00
 *  Author: maczijewski
 */ 


#include "led.h"


void led_set_state(uint32_t tick, spi_mosi_data_t* spi_mosi_data) {
	CLEAR_BIT(PORTC, 0);
	CLEAR_BIT(PORTC, 1);
	CLEAR_BIT(PORTC, 2);
	CLEAR_BIT(PORTC, 7);
	
	
	if(   spi_mosi_data->LED1_period_ticks  != 0
	   && spi_mosi_data->LED1_enabled_ticks != 0
	   && tick % spi_mosi_data->LED1_period_ticks < spi_mosi_data->LED1_enabled_ticks)
	{
		SET_BIT(PORTC, 0);
	}
	
	
	if(   spi_mosi_data->LED2_period_ticks  != 0
	   && spi_mosi_data->LED2_enabled_ticks != 0
	   && tick % spi_mosi_data->LED2_period_ticks < spi_mosi_data->LED2_enabled_ticks)
	{
		SET_BIT(PORTC, 1);
	}
	
	
	if(   spi_mosi_data->LED3_period_ticks  != 0
	   && spi_mosi_data->LED3_enabled_ticks != 0
	   && tick % spi_mosi_data->LED3_period_ticks < spi_mosi_data->LED3_enabled_ticks)
	{
		SET_BIT(PORTC, 2);
	}
	
	if(   spi_mosi_data->LED4_period_ticks  != 0
	   && spi_mosi_data->LED4_enabled_ticks != 0
	   && tick % spi_mosi_data->LED4_period_ticks < spi_mosi_data->LED4_enabled_ticks)
	{
		SET_BIT(PORTC, 7);
	}
}


void led_setup() {
	SET_BIT(DDRC, 0);
	SET_BIT(DDRC, 1);
	SET_BIT(DDRC, 2);
	SET_BIT(DDRC, 7);
}
