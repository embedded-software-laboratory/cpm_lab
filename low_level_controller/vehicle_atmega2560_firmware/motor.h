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
 * \file motor.h
 * 
 * \brief This module provides an interface for setting up and controlling speed and direction of motor movement.
 *
 * \date 20.09.2018 10:02:02
 * \author maczijewski
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_


#include <stdint.h>

/**
 * \brief Can be used in \link motor_set_direction \endlink to force motor to brake.
 * \ingroup low_level_controller
 */
#define MOTOR_DIRECTION_BRAKE 0

/**
 * \brief Can be used in \link motor_set_direction \endlink to set motor direction to forward.
 * \ingroup low_level_controller
 */
#define MOTOR_DIRECTION_FORWARD 1

/**
 * \brief Can be used in \link motor_set_direction \endlink to set motor direction to backwards.
 * \ingroup low_level_controller
 */
#define MOTOR_DIRECTION_REVERSE 2

/**
 * \brief Set the direction in which the vehicle should move. Three values are possible,
 *        see \link MOTOR_DIRECTION_BRAKE \endlink, \link MOTOR_DIRECTION_FORWARD \endlink,
 *        and \link MOTOR_DIRECTION_REVERSE \endlink.
 * \param direction
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void motor_set_direction(uint8_t direction);

/**
 * \brief Allows to set the duty of the motor. Direction of movement depends on \link motor_set_direction \endlink.
 * \param duty PWM signal within a range of [0,400]
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void motor_set_duty(uint16_t duty);

/**
 * \brief Sets up all relevant registers such that the motor can be controlled afterwards.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void motor_setup();


#endif /* MOTOR_H_ */