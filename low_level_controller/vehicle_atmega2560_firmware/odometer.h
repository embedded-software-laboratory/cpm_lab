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
 * \file odometer.h
 *
 * \date 20.09.2018 17:43:13
 * \author: maczijewski
 * 
 * \brief This module provides an interface for setting up and reading values of the odometer and inferred values.
 * 
 * For implementation details see \link odometer.c \endlink.
 */ 


#ifndef ODOMETER_H_
#define ODOMETER_H_

/**
 * \brief Current speed of the vehicle. Inferred by using the last few odometer steps and
 *        the time between these subsequent measurements.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
int16_t get_speed();

/**
 * \brief Returns the current count of odometer steps. Note: Overflow after about 6705km ;)
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
int32_t get_odometer_count();

/**
 * \brief Sets up all relevant registers such that the odometer provides correct values.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void odometer_setup();


#endif /* ODOMETER_H_ */