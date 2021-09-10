/*
 * vehicle_atmega2560_firmware.c
 */

/**
 * \file main.c
 *
 * \author maczijewski
 * \date Created: 17.09.2018 13:43:43
 * 
 * \brief This is the main loop of the low_level_controller.
 * 
 * The low_level_controller may have three internal states:
 * 		  1. The "usual" one, in which it is regularily communicating with the
 * 			mid_level_controller and working on its behalf.
 * 		  2. The safe-mode, which is activated on startup until mid_level_controller has
 * 			finished its boot process and additionally by
 * 			\link watchdog.h \endlink in case that communication with mid_level_controller
 * 			takes to long (e.g. because of a reboot of the mid_level_controller or a not
 * 			working SPI-slave-select-line).
 * 		  3. As long as the low_level_control is in safe-mode it is possible to enable
 * 			\link tests.h \endlink by using a jumper on the main board of the vehicle.
 * 
 * \ingroup low_level_controller
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
#include "tests.h"


int main(void)
{
	sei();
	
	watchdog_disable(); // allow for longer setup procedures
	
	twi_init();
	motor_setup();
	odometer_setup();
	spi_setup();
	servo_timer_setup();
	led_setup();
	adc_setup();
	const bool imu_init_status = imu_setup(); // must be after twi_init()
	crcInit();
	tests_setup();
	
	
	spi_miso_data_t spi_miso_data;
	memset(&spi_miso_data, 0, sizeof(spi_miso_data_t));

	spi_mosi_data_t spi_mosi_data;
	memset(&spi_mosi_data, 0, sizeof(spi_mosi_data_t));
	
	watchdog_enable();	
	watchdog_reset();

    while (1)
    {
	    // All zeros is a safe command (motor stopped, steering centered)
	    memset(&spi_mosi_data, 0, sizeof(spi_mosi_data_t));
		
		if(safe_mode_flag) 
		{
            if ((PINA & 1) == 0) { // test mode
			    tests_apply(&spi_miso_data, &spi_mosi_data);
            }

			_delay_ms(1);
		}
		else
		{	
			spi_exchange(&spi_miso_data, &spi_mosi_data);
		}
		
			
		/// read sensors
			
		// read odometer
		const int16_t speed = get_speed();
		const int32_t odometer_count = get_odometer_count();
		
		// read ADC
		uint16_t battery_voltage = 0;
		uint16_t current_sense = 0;
		adc_measure(&battery_voltage, &current_sense);
		
		// read IMU
		uint16_t imu_yaw = 0;
		int16_t imu_yaw_rate = 0;
		int16_t imu_acceleration_forward = 0;
		int16_t imu_acceleration_left = 0;
		int16_t imu_acceleration_up = 0;
		
		bool imu_status = false;
		if (imu_init_status){
			imu_status = imu_read(		
				&imu_yaw,
				&imu_yaw_rate,
				&imu_acceleration_forward,
				&imu_acceleration_left,
				&imu_acceleration_up
			);
		}
		
		// collect sensor data
		spi_miso_data.odometer_steps = odometer_count;
		spi_miso_data.speed = speed;
		spi_miso_data.battery_voltage = battery_voltage;
		spi_miso_data.motor_current = current_sense;		
		
		if(imu_init_status && imu_status) {
			CLEAR_BIT(spi_miso_data.status_flags, 0);
			spi_miso_data.imu_yaw = imu_yaw;
			spi_miso_data.imu_yaw_rate = imu_yaw_rate;
			spi_miso_data.imu_acceleration_forward = imu_acceleration_forward;
			spi_miso_data.imu_acceleration_left = imu_acceleration_left;
			spi_miso_data.imu_acceleration_up = imu_acceleration_up;
			CLEAR_BIT(spi_miso_data.status_flags, 0);
		}
		else {
			SET_BIT(spi_miso_data.status_flags, 0);
		}


		//// motor derating to limit max speed
		// speed units: 1 m/s ~ 1343
		// speed units: 2 m/s ~ 2686
		if(spi_mosi_data.motor_mode == SPI_MOTOR_MODE_FORWARD 
			&& speed > 2600
			&& spi_mosi_data.motor_pwm > 0)
		{
			int16_t delta_motor_pwm = (speed - 2600)/2;
			if(spi_mosi_data.motor_pwm > delta_motor_pwm) spi_mosi_data.motor_pwm -= delta_motor_pwm;
			else spi_mosi_data.motor_pwm = 0;
		}
		else if(spi_mosi_data.motor_mode == SPI_MOTOR_MODE_REVERSE 
			&& speed < -2600
			&& spi_mosi_data.motor_pwm > 0)
		{
			int16_t delta_motor_pwm = (-speed - 2600)/2;
			if(spi_mosi_data.motor_pwm > delta_motor_pwm) spi_mosi_data.motor_pwm -= delta_motor_pwm;
			else spi_mosi_data.motor_pwm = 0;
		}
		
	
		/// apply commands
		motor_set_direction(spi_mosi_data.motor_mode);
		motor_set_duty(spi_mosi_data.motor_pwm);

		// move the input into the PWM range
		set_servo_pwm(spi_mosi_data.servo_command + 3000);
        led_set_state(spi_mosi_data.vehicle_id);
	}
	
}
