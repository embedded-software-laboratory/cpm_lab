/*
 * vehicle_atmega2560_firmware.c
 *
 * Created: 17.09.2018 13:43:43
 * Author : maczijewski
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "util.h"
#include "led.h"
#include "motor.h"
#include "odometer.h"


volatile int32_t speed = 0; // just for testing

int main(void)
{
	
	SET_BIT(DDRC, 0); // Red LED
	SET_BIT(DDRC, 1); // Green LED
	SET_BIT(DDRC, 2); // Blue LED
	
	motor_setup();
	odometer_setup();
	
	motor_set_direction(MOTOR_DIRECTION_REVERSE);
	
	sei();
	
	
    while (1) 
    {
		motor_set_duty(120);		
		
		speed = get_speed();
		
	    _delay_ms(100);
    }
}