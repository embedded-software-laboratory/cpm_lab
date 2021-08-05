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
 * \file spi_packets.h
 *
 * \date Created: 24.09.2018 18:27:01
 * \author: maczijewski
 * 
 * \ingroup low_level_controller
 */ 


#ifndef SPI_PACKETS_H_
#define SPI_PACKETS_H_


#include <stdint.h>


#define SPI_MOTOR_MODE_BRAKE 0
#define SPI_MOTOR_MODE_FORWARD 1
#define SPI_MOTOR_MODE_REVERSE 2

#define SPI_BUFFER_SIZE 27

/**
 * \brief Struct for the SPI data sent from mid_level_controller to low_level_controller.
 * 	      The struct contains 12 bytes overall.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
typedef struct
{ 
	//! Currently, no purpose?! TODO
	uint32_t pi_tick;
	//! Target PWM signal of motor
	int16_t motor_pwm;
	//! Target command for servo
	int16_t servo_command;
	//! CRC of this package
	uint16_t CRC;
	//! Motor mode (brake, forward, reverse)
	uint8_t motor_mode;
	//! ID of vehicle
	uint8_t vehicle_id;
} __attribute__((packed)) spi_mosi_data_t;

/**
 * \brief Struct for the SPI data sent from low_level_controller to mid_level_controller.
 * 	      The struct contains 27 bytes overall.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
typedef struct
{
	//! Currently, no purpose?! TODO
	uint32_t tick;
	//! Current steps of odometer
	int32_t odometer_steps;
	//! Accumulated yaw based on yaw rate relativ to initial orientation of vehicle
	uint16_t imu_yaw;
	//! Yaw rate provided by IMU
	int16_t imu_yaw_rate;
	//! Acceleration in forward direction provided by IMU
	int16_t imu_acceleration_forward;
	//! Acceleration to the left provided by IMU
	int16_t imu_acceleration_left;
	//! Upward acceleration provided by IMU
	int16_t imu_acceleration_up;
	//! Speed based on odometer
	int16_t speed;
	//! Current battery voltage
	uint16_t battery_voltage;
	//! Current motor current
	uint16_t motor_current;
	//! CRC of this package
	uint16_t CRC;
	/**
	 * \brief Flags indicating the status of the vehicle and its electronic components.
	 * 
	 * Bit 7: reserved |
	 * Bit 6: reserved |
	 * Bit 5: reserved |
	 * Bit 4: reserved |
	 * Bit 3: reserved |
	 * Bit 2: reserved |
	 * Bit 1: SPI status, 0 -> previous mosi packet correctly received, 1 -> fault |
	 * Bit 0: IMU status, 0 -> OK, 1 -> fault
	 * 
	 * Note: reserved=?currently not in use? TODO
	*/
	uint8_t status_flags;
} __attribute__((packed)) spi_miso_data_t;


#ifdef __cplusplus
static_assert(sizeof(spi_mosi_data_t) == 12, "spi_mosi_data_t unexpected size, not packed?");
static_assert(sizeof(spi_miso_data_t) == 27, "spi_miso_data_t unexpected size, not packed?");
static_assert(sizeof(spi_mosi_data_t) <= SPI_BUFFER_SIZE, "SPI buffer too small");
static_assert(sizeof(spi_miso_data_t) <= SPI_BUFFER_SIZE, "SPI buffer too small");
#else
_Static_assert(sizeof(spi_mosi_data_t) == 12, "spi_mosi_data_t unexpected size, not packed?");
_Static_assert(sizeof(spi_miso_data_t) == 27, "spi_miso_data_t unexpected size, not packed?");
_Static_assert(sizeof(spi_mosi_data_t) <= SPI_BUFFER_SIZE, "SPI buffer too small");
_Static_assert(sizeof(spi_miso_data_t) <= SPI_BUFFER_SIZE, "SPI buffer too small");
#endif


#endif /* SPI_PACKETS_H_ */