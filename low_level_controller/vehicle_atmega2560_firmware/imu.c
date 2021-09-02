/**
 * \file imu.c
 *
 * \author maczijewski
 * \date Created: 27.09.2018 10:54:21
 * 
 * \brief This module provides an interface for setting up and reading values of the inertial measurement unit (IMU).
 * 
 * The vehicles use the BNO055 as IMU. This IMU provides several operating modes which can be separated into two general
 * ones: fusion modes and non-fusion modes. In non-fusion modes just the raw signals of the sensors are provided while in
 * fusion modes the internal chip combines these readings for information on relative and absolute orientation. Furthermore,
 * the chip tries to improve the sensor data, for example, it uses the magnetometer to compensate a possible drift of the
 * gyroscope. Since the magnetometer is comparably slow especially in the beginning, this compensation leads to negative
 * side effects on our current implementation. Consequently, we use the non-fusion mode ACCGYRO where only accelerometer
 * and gyroscope are activated.
 * 
 * NOTE: The IMU provides two pages. Dependend on which page is selected, the same address provides access to different registers!
 * 
 * 
 * \ingroup low_level_controller
 */ 

/**
 * \brief TWI address of IMU
 * \ingroup low_level_controller
 */
#define BNO055_ADDRESS (0x28)

/**
 * \brief Register address of the identification code of the IMU
 * \ingroup low_level_controller
 */
#define BNO055_CHIP_ID_ADDR  0x00

/**
 * \brief Identification code of the IMU
 * \ingroup low_level_controller
 */
#define BNO055_CHIP_ID      (0xA0)

/**
 * \brief Address of the configuration register in which either page 0 or page 1 can be selected.
 * \ingroup low_level_controller
 */
#define BNO055_REGISTER_PAGE_ADDR (0x07)

/**
 * \brief On page 1, this is the address of the accelerometer configuration register. This register allows to specify the power
 * 			mode, the bandwidth, and the range of the accelerometer.
 * \ingroup low_level_controller
 */
#define BNO055_ACC_CONFIG_ADDR (0x08)

/**
 * \brief On page 1, this is the address of the first gyroscope configuration register. This register allows to specify the bandwidth
 * 			and the range of the gyroscope.
 * \ingroup low_level_controller
 */
#define BNO055_GYRO_CONFIG0_ADDR (0x0A)

/**
 * \brief On page 1, this is the address of the second gyroscope configuration register. This register allows to specify the power
 * 			mode of the gyroscope.
 * \ingroup low_level_controller
 */
#define BNO055_GYRO_CONFIG1_ADDR (0x0B)

/**
 * \brief On page 0, this is the address of the configuration register, where the operation mode can be selected.
 * \ingroup low_level_controller
 */
#define BNO055_OPR_MODE_ADDR     0X3D

/**
 * \brief On page 0, this is the address of the configuration register, where the power mode can be selected.
 * \ingroup low_level_controller
 */
#define BNO055_PWR_MODE_ADDR     0X3E

/**
 * \brief This is the value of the normal power mode.
 * \ingroup low_level_controller
 */
#define BNO055_POWER_MODE_NORMAL        0X00

/**
 * \brief This is the value of the fusion operation mode which provides absolute orientation and activates all sensors.
 * \ingroup low_level_controller
 */
#define BNO055_OPERATION_MODE_NDOF      0X0C

/**
 * \brief This is the value of the fusion operation mode which provides relative orientation and activates the accelerometer and gyroscope.
 * \ingroup low_level_controller
 */
#define BNO055_OPERATION_MODE_IMU      0b00001000

/**
 * \brief This is the value of the non-fusion operation mode which just activates the accelerometer and gyroscope.
 * \ingroup low_level_controller
 */
#define BNO055_OPERATION_MODE_ACCGYRO      0b00000101

/**
 * \brief On page 0, this is the address of the register in which the lower byte of the x-axis linear acceleration data is provided.
 * \ingroup low_level_controller
 */
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR  0X28

/**
 * \brief On page 0, this is the address of the register in which the lower byte of the x-axis acceleration data is provided.
 * \ingroup low_level_controller
 */
#define BNO055_ACCEL_DATA_X_LSB_ADDR         0X08

/**
 * \brief On page 0, this is the address of the register in which the lower byte of the x-axis gyroscope data is provided.
 * \ingroup low_level_controller
 */
#define BNO055_GYRO_DATA_X_LSB_ADDR         0X14

/**
 * \brief On page 0, this is the address of the register in which the lower byte of the y-axis gyroscope data is provided.
 * \ingroup low_level_controller
 */
#define BNO055_GYRO_DATA_Y_LSB_ADDR         0X16

/**
 * \brief On page 0, this is the address of the register in which the lower byte of the z-axis gyroscope data is provided.
 * \ingroup low_level_controller
 */
#define BNO055_GYRO_DATA_Z_LSB_ADDR         0X18

/**
 * \brief On page 0, this is the address of the register with which system calls can be triggered, e.g., to reset the system.
 * \ingroup low_level_controller
 */
#define BNO055_SYS_TRIGGER_ADDR              0X3F

/**
 * \brief This is the value which stimulates a system reset of the IMU.
 * \ingroup low_level_controller
 */
#define BNO055_SYS_TRIGGER_RESET_SYSTEM  0b00100000


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include "twi.h"
#include "util.h"


bool imu_setup() {
	
	uint8_t buffer[10];
	uint8_t status = 0;
	
	_delay_ms(650); // Wait for the IMU to boot	
	
	// check chip ID
	buffer[0] = BNO055_CHIP_ID_ADDR;
	if(twi_writeTo(BNO055_ADDRESS, buffer, 1, true, false) != 0) {return false;}
	if(twi_readFrom(BNO055_ADDRESS, buffer, 1, true) != 1) {return false;}
	if(buffer[0] != BNO055_CHIP_ID) {return false;}
	_delay_ms(10);
	
	
	// Reset IMU
	buffer[0] = BNO055_SYS_TRIGGER_ADDR;
	buffer[1] = BNO055_SYS_TRIGGER_RESET_SYSTEM;
	status = twi_writeTo(BNO055_ADDRESS, buffer, 2, true, true);
	if(status != 0) {
		return false;
	}
	_delay_ms(650); // Wait for the IMU to boot
		
		
	// set power mode
	buffer[0] = BNO055_PWR_MODE_ADDR;
	buffer[1] = BNO055_POWER_MODE_NORMAL;
	status = twi_writeTo(BNO055_ADDRESS, buffer, 2, true, true);
	if(status != 0) {
		return false;
	}
	_delay_ms(10);


	// set operation mode
	buffer[0] = BNO055_OPR_MODE_ADDR;
	buffer[1] = BNO055_OPERATION_MODE_ACCGYRO;
	status = twi_writeTo(BNO055_ADDRESS, buffer, 2, true, true);
	if(status != 0) {
		return false;
	}
	_delay_ms(10);
	
	
	// activate register page 1
	buffer[0] = BNO055_REGISTER_PAGE_ADDR;
	buffer[1] = 1;
	status = twi_writeTo(BNO055_ADDRESS, buffer, 2, true, true);
	if(status != 0) {
		return false;
	}
	_delay_ms(10);
	
	// gyro config 0
	buffer[0] = BNO055_GYRO_CONFIG0_ADDR;
	buffer[1] = 0b00010000; // range 2000 dps, bandwidth 116 Hz
	status = twi_writeTo(BNO055_ADDRESS, buffer, 2, true, true);
	if(status != 0) {
		return false;
	}
	_delay_ms(10);
	
	
	// activate register page 0
	buffer[0] = BNO055_REGISTER_PAGE_ADDR;
	buffer[1] = 0;
	status = twi_writeTo(BNO055_ADDRESS, buffer, 2, true, true);
	if(status != 0) {
		return false;
	}
	_delay_ms(10);


	return true;	
}

/**
 * \brief Variable to accumulate the yaw_rate of the IMU in order to get the yaw.
 * \author maczijewski
 * \ingroup low_level_controller
 */
static int32_t imu_yaw_accumulator = 0;


bool imu_read(
	uint16_t* imu_yaw,
	int16_t* imu_yaw_rate, 
	int16_t* imu_acceleration_forward,
	int16_t* imu_acceleration_left,
	int16_t* imu_acceleration_up
)
{	
	bool success_flag = true;
	uint8_t buffer[10];
	
	// read yaw rate
	buffer[0] = BNO055_GYRO_DATA_Z_LSB_ADDR;
	if(twi_writeTo(BNO055_ADDRESS, buffer, 1, true, false) != 0) success_flag = false;
	if(twi_readFrom(BNO055_ADDRESS, buffer, 2, true) != 2) success_flag = false;
	*imu_yaw_rate = *((int16_t*)(buffer));
	
	_delay_us(50);
	
	// read acceleration
	buffer[0] = BNO055_ACCEL_DATA_X_LSB_ADDR;
	if(twi_writeTo(BNO055_ADDRESS, buffer, 1, true, false) != 0) success_flag = false;
	if(twi_readFrom(BNO055_ADDRESS, buffer, 6, true) != 6) success_flag = false;
	
	*imu_acceleration_left = *((int16_t*)(buffer));
	*imu_acceleration_forward = -*((int16_t*)(buffer+2));
	*imu_acceleration_up = *((int16_t*)(buffer+4));
	
	
	imu_yaw_accumulator += *imu_yaw_rate;
	if(imu_yaw_accumulator < 0)
	{
		// 288000 = 360*16*50, corresponds to 1 rotation at 50 Hz integration rate
		imu_yaw_accumulator += 288000;
	}
	else if(imu_yaw_accumulator > 288000)
	{
		imu_yaw_accumulator -= 288000;
	}
	
	// Divide by 32 to get back into the 16 bit range.
	*imu_yaw = (uint16_t)(imu_yaw_accumulator/32);
	
	return success_flag;
}