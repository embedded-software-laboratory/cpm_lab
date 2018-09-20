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
#include "motor.h"

int main(void)
{
	
	SET_BIT(DDRC, 0); // LED
	
	motor_setup();
	
	
	sei();
	
	uint16_t duty = 0;
	
    while (1) 
    {
		duty = (duty+1)%400;
		motor_set_duty(duty);
			
		TOGGLE_BIT(PORTC, 0); // blink LED
	    _delay_ms(50);
    }
}

