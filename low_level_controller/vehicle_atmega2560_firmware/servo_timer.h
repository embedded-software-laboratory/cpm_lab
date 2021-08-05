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
 * \file servo_timer.h
 *
 * \date 24.09.2018 16:18:47
 * \author: maczijewski
 * 
 * \brief This module provides an interface for setting up and controlling the servo via PWM,
 *        where the servo PWM signal runs at 50 Hz
 */ 


#ifndef SERVO_TIMER_H_
#define SERVO_TIMER_H_


#include <stdint.h>


/**
 * \brief Returns the time since chip startup, in 20 msec increments.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
uint32_t get_tick();

/**
 * \brief Sets up all relevant registers such that the pwm for the servo works properly.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void servo_timer_setup();

/**
 * \brief Sets the position of the servo by providing a PWM value in the range [1800,4200]
 *        where 3000 is the middle value where the vehicle drives straight.
 * \param pwm
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void set_servo_pwm(uint16_t pwm);


#endif /* SERVO_TIMER_H_ */