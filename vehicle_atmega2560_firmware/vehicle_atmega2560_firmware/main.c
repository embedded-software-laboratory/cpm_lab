/*
 * vehicle_atmega2560_firmware.c
 *
 * Created: 17.09.2018 13:43:43
 * Author : maczijewski
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdint.h>
#include "util.h"
#include "led.h"
#include "motor.h"
#include "odometer.h"
#include "spi.h"
#include "servo_timer.h"
#include "adc.h"
#include "twi.h"
#include "imu.h"
#include "crc.h"
#include "watchdog.h"

#define TEST_MODE TEST_NONE


int main(void)
{
	sei();
	
	watchdog_disable(); // allow for longer setup procedures
	
	led_setup();
	twi_init();
	motor_setup();
	odometer_setup();
	spi_setup();
	servo_timer_setup();
	adc_setup();
	const bool imu_init_status = imu_setup(); // must be after twi_init()
	crcInit();
	
	// stack spi variables
	spi_miso_data_t spi_miso_data;
	memset(&spi_miso_data, 0, sizeof(spi_miso_data_t));

	spi_mosi_data_t spi_mosi_data;
	memset(&spi_mosi_data, 0, sizeof(spi_mosi_data_t));
	
	watchdog_enable(); // interrupt mode
	
	watchdog_reset();


    while (1)
    {
		if(safe_mode_flag) { // enter safe operation mode
			// reset spi variables
			memset(&spi_miso_data, 0, sizeof(spi_miso_data_t));
			memset(&spi_mosi_data, 0, sizeof(spi_mosi_data_t));
			
			safe_mode(&spi_mosi_data);
		}
		else { // normal operation mode
			// 1. spi transmission
			memset(&spi_mosi_data, 0, sizeof(spi_mosi_data_t)); // reset spi mosi i.e. to be received before data exchange
				
			spi_exchange(&spi_miso_data, &spi_mosi_data);
			
			
			// 2. reset watchdog
			watchdog_reset();
			
			
			// 3. sensor DAQ
			// read odometer
			const int16_t speed = get_speed();
			const int32_t odometer_count = get_odometer_count();
		
			// read adc
			uint16_t battery_voltage = 0;
			uint16_t current_sense = 0;
			adc_measure(&battery_voltage, &current_sense);
		
			// read imu
			uint16_t imu_yaw = 0;
			int16_t imu_yaw_rate = 0;
			int16_t imu_acceleration_forward = 0;
			int16_t imu_acceleration_left = 0;
			int16_t imu_acceleration_up = 0;
		
			const bool imu_status = imu_read(		
				&imu_yaw,
				&imu_yaw_rate,
				&imu_acceleration_forward,
				&imu_acceleration_left,
				&imu_acceleration_up
			);
		
			// collect sensor data
			spi_miso_data.odometer_steps = odometer_count;
			spi_miso_data.speed = speed;
			spi_miso_data.battery_voltage = battery_voltage;
			spi_miso_data.motor_current = current_sense;		
		
			if(imu_init_status && imu_status) {
				spi_miso_data.imu_yaw = imu_yaw;
				spi_miso_data.imu_yaw_rate = imu_yaw_rate;
				spi_miso_data.imu_acceleration_forward = imu_acceleration_forward;
				spi_miso_data.imu_acceleration_left = imu_acceleration_left;
				spi_miso_data.imu_acceleration_up = imu_acceleration_up;
			}
			else {
				SET_BIT(spi_miso_data.status_flags, 0);
			}		
		
	
			// 4. apply commands
			motor_set_direction(spi_mosi_data.motor_mode);
			motor_set_duty(spi_mosi_data.motor_pwm);
			set_servo_pwm(spi_mosi_data.servo_command + 3000);
			led_set_state(&spi_mosi_data);
		}
	}
	
}