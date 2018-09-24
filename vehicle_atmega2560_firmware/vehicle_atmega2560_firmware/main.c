/*
 * vehicle_atmega2560_firmware.c
 *
 * Created: 17.09.2018 13:43:43
 * Author : maczijewski
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "util.h"
#include "led.h"
#include "motor.h"
#include "odometer.h"
#include "spi.h"
#include "servo_timer.h"


int main(void)
{
	
	SET_BIT(DDRC, 0); // Red LED
	SET_BIT(DDRC, 1); // Green LED
	SET_BIT(DDRC, 2); // Blue LED
	
	motor_setup();
	odometer_setup();
	spi_setup();
	servo_timer_setup();
	
	motor_set_direction(MOTOR_DIRECTION_FORWARD);
	
	sei();
	
	
    while (1) 
    {
		spi_mosi_data_t spi_mosi_data;
		spi_miso_data_t spi_miso_data; // TODO default init
		
		spi_receive(&spi_mosi_data);
		// TODO validate spi_mosi_data CRC, default init if invalid
		
	    const uint32_t tick = get_tick();
		const int32_t speed = get_speed();
		
		motor_set_duty(5);
		
		
		//spi_send(speed);
		uint16_t timer = TCNT1; // just for testing
		
		/*
		DISABLE_RED_LED;
		DISABLE_GREEN_LED;
		DISABLE_BLUE_LED;
		
		if(speed > 0) {
			ENABLE_RED_LED;
		}
		if(speed > 500000) {
			ENABLE_GREEN_LED;
		}
		if(speed > 800000) {
			ENABLE_BLUE_LED;
		}*/
		
		TOGGLE_BLUE_LED;
		
		
		// TODO calculate spi_miso_data CRC
		spi_send(&spi_miso_data);
		
		tick_wait(); // synchronize 50 Hz loop
    }
}