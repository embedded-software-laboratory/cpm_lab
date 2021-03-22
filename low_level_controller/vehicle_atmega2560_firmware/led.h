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
 * led.h
 *
 * Created: 20.09.2018 20:24:06
 *  Author: maczijewski
 */ 


#ifndef LED_H_
#define LED_H_


#include "servo_timer.h"
#include "spi_packets.h"

/**
 * \brief TODO
 * \param vehicle_id_in
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void led_set_state(uint8_t vehicle_id_in);

/**
 * \brief TODO
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void led_toggle();

/**
 * \brief TODO
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void led_setup();

/**
 * \brief TODO
 * \param led1
 * \param led2
 * \param led3
 * \param led4
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void led_test(uint8_t led1, uint8_t led2, uint8_t led3, uint8_t led4);


#endif /* LED_H_ */