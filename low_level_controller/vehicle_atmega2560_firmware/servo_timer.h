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
 * servo_timer.h
 *
 * Created: 24.09.2018 16:18:47
 *  Author: maczijewski
 */ 


#ifndef SERVO_TIMER_H_
#define SERVO_TIMER_H_


#include <stdint.h>


// The servo PWM signal runs at 50Hz

/**
 * \brief TODO
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
uint32_t get_tick(); // time since chip startup, in 20 msec increments

/**
 * \brief TODO
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void servo_timer_setup();

/**
 * \brief TODO
 * \param pwm
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void set_servo_pwm(uint16_t pwm); // values from 2000 to 4000. center at 3000


#endif /* SERVO_TIMER_H_ */