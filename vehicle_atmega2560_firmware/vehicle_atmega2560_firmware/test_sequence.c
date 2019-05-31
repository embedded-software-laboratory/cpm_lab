/*
 * test_sequence.c
 *
 * Created: 05.31.2019 11:32:19
 *  Author: cfrauzem
 */ 

#include <stdint.h>
#include "spi.h"
#include "test_sequence.h"





static void led_test(spi_mosi_data_t* spi_mosi_data, spi_miso_data_t* spi_miso_data) {
	// LED test
	// blink LEDs 1-4 slow to fast respectively
	// need to overwrite default settings
	spi_mosi_data->LED1_period_ticks = 1;
	spi_mosi_data->LED1_enabled_ticks = 0;
	spi_mosi_data->LED2_period_ticks = 1;
	spi_mosi_data->LED2_enabled_ticks = 0;
	spi_mosi_data->LED3_period_ticks = 1;
	spi_mosi_data->LED3_enabled_ticks = 0;
	spi_mosi_data->LED4_period_ticks = 1;
	spi_mosi_data->LED4_enabled_ticks = 0;

	uint32_t tick;

	// 50 ticks per second
	tick = spi_miso_data->tick;

	// blink LED 1 1X
	if(tick%500<50) {
		spi_mosi_data->LED1_period_ticks = 50;
		spi_mosi_data->LED1_enabled_ticks = 25;
	}
	// blink LED 2 2X
	else if(tick%500<150) {
		spi_mosi_data->LED2_period_ticks = 50;
		spi_mosi_data->LED2_enabled_ticks = 25;
	}
	// blink LED 2 2X
	else if(tick%500<300) {
		spi_mosi_data->LED3_period_ticks = 50;
		spi_mosi_data->LED3_enabled_ticks = 25;
	}
	// blink LED 4 4X
	else {
		spi_mosi_data->LED4_period_ticks = 50;
		spi_mosi_data->LED4_enabled_ticks = 25;
	}
}


static void servo_current_test(spi_mosi_data_t* spi_mosi_data, spi_miso_data_t* spi_miso_data) {
	// servo current measurement loop
	// measure servo current consumption
	// max current at velocity=0
	uint32_t tick;
	uint8_t period	= 4;
	uint8_t on		= 2;

	// 50 ticks per second
	tick = spi_miso_data->tick;

	if(tick%(50*period)>50*on) {
		spi_mosi_data->servo_command = 800;		// +800, -800
	}
	else {
		spi_mosi_data->servo_command = -800;	// +800, -800
	}
}


static void servo_enable_test(spi_mosi_data_t* spi_mosi_data, spi_miso_data_t* spi_miso_data) {
	// servo enable test loop
	// 1. servo hum for 2sec
	// 2. servo disable till reactivated at t=3sec
	// 3. reset at t=4sec
	uint32_t tick;
	uint8_t period	= 4;
	uint8_t on		= 3;

	// 50 ticks per second
	tick = spi_miso_data->tick;

	// servo command translated into pwm
	// pwm = servo_command + 3000
	if(tick%(50*period)>50*on) {
	spi_mosi_data->servo_command = 1;
	}
	else {
	spi_mosi_data->servo_command = 0;
	}
}


static void system_test(spi_mosi_data_t* spi_mosi_data, spi_miso_data_t* spi_miso_data) {
	// systems test program
	spi_mosi_data->motor_mode = SPI_MOTOR_MODE_FORWARD;
	spi_mosi_data->motor_pwm = 60;

	// steer against yaw
	int32_t yaw = (int32_t)(spi_miso_data->imu_yaw);
	if(yaw > 4500) yaw -= 9000;

	spi_mosi_data->servo_command = 2*yaw;

	if(spi_mosi_data->servo_command >  400) spi_mosi_data->servo_command =  400;
	if(spi_mosi_data->servo_command < -400) spi_mosi_data->servo_command = -400;

	// motor + LEDs
	int32_t s = spi_miso_data->odometer_steps;
	
	//
	spi_mosi_data->LED1_period_ticks = 1;
	spi_mosi_data->LED1_enabled_ticks = 0;
	spi_mosi_data->LED2_period_ticks = 1;
	spi_mosi_data->LED2_enabled_ticks = 0;
	spi_mosi_data->LED3_period_ticks = 1;
	spi_mosi_data->LED3_enabled_ticks = 0;
	spi_mosi_data->LED4_period_ticks = 1;
	spi_mosi_data->LED4_enabled_ticks = 0;

	if(s&1) spi_mosi_data->LED1_enabled_ticks = 1;
	if(s&2) spi_mosi_data->LED2_enabled_ticks = 1;
	if(s&4) spi_mosi_data->LED3_enabled_ticks = 1;
	if(s&8) spi_mosi_data->LED4_enabled_ticks = 1;

	if(s>0) spi_mosi_data->motor_mode = SPI_MOTOR_MODE_REVERSE;
	if(s<0) spi_mosi_data->motor_mode = SPI_MOTOR_MODE_FORWARD;

	if(s<0) s = -s;
	if(s > 200) s = 200;
	spi_mosi_data->motor_pwm = s;
}



void test_sequence(spi_mosi_data_t* spi_mosi_data, spi_miso_data_t* spi_miso_data, TestSequenceName testSequenceName)
{
	switch(testSequenceName) {
		case TEST_LED: led_test(spi_mosi_data, spi_miso_data); break;
		case TEST_SERVO_CURRENT: servo_current_test(spi_mosi_data, spi_miso_data); break;
		case TEST_SERVO_ENABLE: servo_enable_test(spi_mosi_data, spi_miso_data); break;
		case TEST_SYSTEM: system_test(spi_mosi_data, spi_miso_data); break;
		default: break;
	}
	
}