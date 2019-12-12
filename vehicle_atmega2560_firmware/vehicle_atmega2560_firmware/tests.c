/*
 * tests.c
 *
 * Created: 09/08/2019 09:48:04
 *  Author: Janis
 */ 


#include "tests.h"
#include "util.h"
#include "watchdog.h"
#include "led.h"


void tests_setup()
{
	// The extension pins are used to enable the tests with a jumper
	
	// Pin A0 is input pullup
	// Pin A3 is output low
	// Connecting A0 to A4 will pull A0 low
	SET_BIT(PORTA, 0);
	CLEAR_BIT(DDRA, 0);
	CLEAR_BIT(PORTA, 4);
	SET_BIT(DDRA, 4);
	
	
}

void tests_apply(uint32_t tick, spi_miso_data_t *packet_send, spi_mosi_data_t *packet_received)
{
	if((PINA & 1) == 0) // Test Pin A0 low
	{
		uint8_t led1 = packet_send->odometer_steps & 1;
		uint8_t led2 = (packet_send->odometer_steps >> 1) & 1;
		uint8_t led3 = (packet_send->odometer_steps >> 2) & 1;
		uint8_t led4 = (packet_send->odometer_steps >> 3) & 1;
		
		test_led(led1, led2, led3, led4);
		
		packet_received->motor_mode = SPI_MOTOR_MODE_BRAKE;
		if(packet_send->odometer_steps > 5) 
		{
			packet_received->motor_mode = SPI_MOTOR_MODE_REVERSE;
			packet_received->motor_pwm = 2*packet_send->odometer_steps;
		}
		if(packet_send->odometer_steps < -5) 
		{
			packet_received->motor_mode = SPI_MOTOR_MODE_FORWARD;
			packet_received->motor_pwm = -2*packet_send->odometer_steps;
		}
		
		int32_t yaw = packet_send->imu_yaw;
		if(yaw > 4500) yaw -= 9000;
		packet_received->servo_command = yaw / 3;
	}
}