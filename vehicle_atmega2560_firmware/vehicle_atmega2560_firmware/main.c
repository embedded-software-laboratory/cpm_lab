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

int main(void)
{
	
	SET_BIT(DDRC, 0); // Red LED
	SET_BIT(DDRC, 1); // Green LED
	SET_BIT(DDRC, 2); // Blue LED
	
	motor_setup();
	odometer_setup();
	spi_setup();
	
	motor_set_direction(MOTOR_DIRECTION_FORWARD);
	
	sei();
	
	
    while (1) 
    {
		
		int32_t speed = get_speed();
		
		motor_set_duty(0);
		
		
		//spi_send(speed);
		uint16_t timer = TCNT1; // just for testing
		spi_send(timer);
		
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
		}
		
		_delay_ms(1000);
    }
}