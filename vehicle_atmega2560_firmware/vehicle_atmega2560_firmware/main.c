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


int main(void)
{
	
	SET_BIT(DDRC, 0); // Red LED
	SET_BIT(DDRC, 1); // Green LED
	SET_BIT(DDRC, 2); // Blue LED
	
	_delay_ms(500); // Wait for the IMU to boot
		
	sei();
	
	twi_init();
	motor_setup();
	odometer_setup();
	spi_setup();
	servo_timer_setup();
	adc_setup();
	const bool imu_init_status = imu_setup(); // must be after twi_init()
	crcInit();
	
	
    while (1) 
    {
	    const uint32_t tick = get_tick();
		
		// Read Odometer
	    const int16_t speed = get_speed();
	    const int32_t odometer_count = get_odometer_count();
		
		// Read ADC
		uint16_t battery_voltage = 0;
		uint16_t current_sense = 0;
		adc_measure(&battery_voltage, &current_sense);
		
		// Read IMU
		uint16_t imu_yaw = 0;
		uint16_t imu_acceleration_forward = 0;
		uint16_t imu_acceleration_left = 0;
		const bool imu_status = imu_read(&imu_yaw, &imu_acceleration_forward, &imu_acceleration_left);
		
		// Read SPI
		uint8_t safe_mode_flag = 0;
		spi_mosi_data_t spi_mosi_data;		
		uint32_t latest_receive_tick = spi_receive(&spi_mosi_data);
		
		if(latest_receive_tick + 5 < tick) { // if we do not receive new data, go into safe mode
			safe_mode_flag = 1;
		}
		
		// Validate SPI CRC
		uint16_t mosi_CRC_actual = spi_mosi_data.CRC;
		spi_mosi_data.CRC = 0;
		uint16_t mosi_CRC_target = crcFast((uint8_t*)(&spi_mosi_data), sizeof(spi_mosi_data_t));
		
		if(mosi_CRC_actual != mosi_CRC_target) {
			safe_mode_flag = 1;
		}
		
		if(safe_mode_flag) {
			memset(&spi_mosi_data, 0, sizeof(spi_mosi_data_t)); // Safe default value: all zeros
		}		
		
		// Apply commands
		motor_set_direction(spi_mosi_data.motor_mode);
		motor_set_duty(spi_mosi_data.motor_pwm);
		set_servo_pwm(spi_mosi_data.servo_command + 3000);
		
		if(spi_mosi_data.LED_bits & 1) ENABLE_RED_LED;
		else DISABLE_RED_LED;		
		
		if((tick >> 5) & 1) ENABLE_GREEN_LED;
		else DISABLE_GREEN_LED;
		
		if((spi_mosi_data.LED_bits >> 2) & 1) ENABLE_BLUE_LED;
		else DISABLE_BLUE_LED;
		
		
		// Send sensor data to master
		spi_miso_data_t spi_miso_data;
		memset(&spi_miso_data, 0, sizeof(spi_miso_data_t));
		spi_miso_data.tick = tick;
		spi_miso_data.odometer_steps = odometer_count;
		spi_miso_data.imu_yaw = imu_yaw;
		spi_miso_data.imu_acceleration_forward = imu_acceleration_forward;
		spi_miso_data.imu_acceleration_left = imu_acceleration_left;
		spi_miso_data.speed = speed;
		spi_miso_data.battery_voltage = battery_voltage;
		spi_miso_data.motor_current = current_sense;		
		
		if(!(imu_init_status && imu_status)) {
			SET_BIT(spi_miso_data.status_flags, 0);
		}		
		
		spi_miso_data.CRC = 0;
		spi_miso_data.CRC = crcFast((uint8_t*)(&spi_miso_data), sizeof(spi_miso_data_t));
		spi_send(&spi_miso_data);
		
		tick_wait(); // synchronize 50 Hz loop
    }
}