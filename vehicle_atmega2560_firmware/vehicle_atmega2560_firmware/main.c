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
#include "adc.h"
#include "i2c.h"
#include <string.h>


int main(void)
{
	
	SET_BIT(DDRC, 0); // Red LED
	SET_BIT(DDRC, 1); // Green LED
	SET_BIT(DDRC, 2); // Blue LED
	
	motor_setup();
	odometer_setup();
	spi_setup();
	servo_timer_setup();
	adc_setup();
	i2c_setup();
	
	sei();
	
	
    while (1) 
    {
	    const uint32_t tick = get_tick();
	    const int16_t speed = get_speed();
	    const int32_t odometer_count = get_odometer_count();
		uint16_t battery_voltage = 0;
		uint16_t current_sense = 0;
		adc_measure(&battery_voltage, &current_sense);
		
		uint8_t safe_mode_flag = 0;
		spi_mosi_data_t spi_mosi_data;
		memset(&spi_mosi_data, 0, sizeof(spi_mosi_data_t));
		
		uint32_t latest_receive_tick = spi_receive(&spi_mosi_data);
		
		// TODO validate spi_mosi_data CRC
		
		if(latest_receive_tick + 5 < tick) { // if we do not receive new data, go into safe mode
			safe_mode_flag = 1;
		}		
		
		if(safe_mode_flag) {
			spi_mosi_data.motor_mode = SPI_MOTOR_MODE_BRAKE;
			spi_mosi_data.motor_pwm = 0;
		}		
		
		motor_set_direction(spi_mosi_data.motor_mode); // TODO speed controller
		motor_set_duty(spi_mosi_data.motor_pwm);
		set_servo_pwm(spi_mosi_data.servo_command + 3000);
		
		if(spi_mosi_data.LED_bits & 1) ENABLE_RED_LED;
		else DISABLE_RED_LED;		
		
		if((spi_mosi_data.LED_bits >> 1) & 1) ENABLE_GREEN_LED;
		else DISABLE_GREEN_LED;
		
		if((spi_mosi_data.LED_bits >> 2) & 1) ENABLE_BLUE_LED;
		else DISABLE_BLUE_LED;
		
		
		spi_miso_data_t spi_miso_data;
		memset(&spi_miso_data, 0, sizeof(spi_miso_data_t));
		spi_miso_data.tick = tick;
		spi_miso_data.odometer_steps = odometer_count;
		spi_miso_data.speed = speed;
		spi_miso_data.battery_voltage = battery_voltage;
		spi_miso_data.motor_current = current_sense;
		spi_miso_data.debugC = -spi_mosi_data.debugA / 2;
		// TODO fill spi_miso_data
		
		// TODO calculate spi_miso_data CRC
		spi_send(&spi_miso_data);
		
		tick_wait(); // synchronize 50 Hz loop
    }
}