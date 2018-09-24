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


volatile int32_t speed = 0; // just for testing
volatile int32_t ctrl_I = 0; // just for testing

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
		
		speed = get_speed();
		
		int32_t Kp_e = (speed - 15000) / (-3000);
		
		if(Kp_e > 0) ctrl_I++;
		if(Kp_e < 0) ctrl_I--;
		
		
		if(ctrl_I > 200) ctrl_I = 200;
		
		if(ctrl_I < -200) ctrl_I = -200;
		
		
		int32_t motor_duty = Kp_e + ctrl_I + 90;
		
		if(motor_duty < 0) motor_duty = 0;
		
		
		
		motor_set_duty(motor_duty);
		
		spi_send_speed(speed);
		//uint16_t timer = TCNT1; // just for testing
		//spi_send_speed(timer);
		
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
		
		_delay_ms(100);
    }
}