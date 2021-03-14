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

/**
 * \file imu.c
 *
 * \author maczijewski
 * \date Created: 27.09.2018 10:54:21
 * 
 * \ingroup low_level_controller
 */ 

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_ADDRESS (0x28)

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_CHIP_ID_ADDR  0x00

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_CHIP_ID      (0xA0)

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_REGISTER_PAGE_ADDR (0x07)

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_ACC_CONFIG_ADDR (0x08) // On page 1 !

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_GYRO_CONFIG0_ADDR (0x0A) // On page 1 !

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_GYRO_CONFIG1_ADDR (0x0B) // On page 1 !

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_EULER_H_LSB_ADDR (0x1A)

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_EULER_H_MSB_ADDR (0x1B)

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_OPR_MODE_ADDR     0X3D

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_PWR_MODE_ADDR     0X3E

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_POWER_MODE_NORMAL        0X00

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_OPERATION_MODE_NDOF      0X0C

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_OPERATION_MODE_IMU      0b00001000

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_OPERATION_MODE_ACCGYRO      0b00000101

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR  0X28

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_ACCEL_DATA_X_LSB_ADDR         0X08

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_GYRO_DATA_X_LSB_ADDR         0X14

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_GYRO_DATA_Y_LSB_ADDR         0X16

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_GYRO_DATA_Z_LSB_ADDR         0X18

/**
 * \brief TODO
 * \ingroup low_level_controller
 */
#define BNO055_SYS_TRIGGER_ADDR              0X3F

/**
 * \brief TODO
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
 * \brief TODO
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
	
	/*
	// read yaw
	buffer[0] = BNO055_EULER_H_LSB_ADDR;
	if(twi_writeTo(BNO055_ADDRESS, buffer, 1, true, false) != 0) success_flag = false;
	if(twi_readFrom(BNO055_ADDRESS, buffer, 2, true) != 2) success_flag = false;
	*imu_yaw = *((uint16_t*)(buffer));
	
	_delay_us(50);
	*/
	
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