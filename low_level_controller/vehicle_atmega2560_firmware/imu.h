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
 * imu.h
 *
 * Created: 27.09.2018 10:54:07
 *  Author: maczijewski
 */ 


#ifndef IMU_H_
#define IMU_H_


#include <stdbool.h>
#include <stdint.h>

/**
 * \brief TODO
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
bool imu_setup();

/**
 * \brief TODO
 * \param imu_yaw
 * \param imu_yaw_rate
 * \param imu_acceleration_forward
 * \param imu_acceleration_left
 * \param imu_acceleration_up
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