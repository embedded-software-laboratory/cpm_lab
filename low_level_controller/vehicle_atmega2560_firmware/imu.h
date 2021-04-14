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
 * \file imu.h
 *
 * \author maczijewski
 * \date Created: 27.09.2018 10:54:07
 * 
 * \brief This module provides an interface for setting up and reading values of the inertial measurement unit (IMU).
 * 
 * For details on the used IMU and the implementation see \link imu.c \endlink.
 * 
 * \ingroup low_level_controller
 */ 


#ifndef IMU_H_
#define IMU_H_


#include <stdbool.h>
#include <stdint.h>

/**
 * \brief Starts IMU and configures its power mode, operation mode, and sensors.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
bool imu_setup();

/**
 * \brief Reads the sensor values currently provided by the IMU.
 * \param imu_yaw value of the yaw relative to the initial orientation. This result is not directly
 * 				  read from the IMU but accumulated based on the yaw_rate.
 * \param imu_yaw_rate rate with which the yaw changes
 * \param imu_acceleration_forward acceleration in forward direction
 * \param imu_acceleration_left acceleration to the left
 * \param imu_acceleration_up acceleration upwards
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
bool imu_read(
	uint16_t* imu_yaw,
	int16_t* imu_yaw_rate,
	int16_t* imu_acceleration_forward,
	int16_t* imu_acceleration_left,
	int16_t* imu_acceleration_up
);


#endif /* IMU_H_ */