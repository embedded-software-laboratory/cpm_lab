// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

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