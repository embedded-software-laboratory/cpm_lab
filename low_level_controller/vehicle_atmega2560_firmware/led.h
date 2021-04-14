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
 * \file led.h
 *
 * \date 20.09.2018 20:24:06
 * \author maczijewski
 * 
 * \brief There are 4 LEDs mounted at the vehicle: 3 for giving the position and one for
 *        presenting the vehicle ID by flashing with an ID-specific pattern/frequency. This
 *        module provides all functions to control these LEDs.
 * 
 * \ingroup low_level_controller
 */ 


#ifndef LED_H_
#define LED_H_


#include "servo_timer.h"
#include "spi_packets.h"

/**
 * \brief This function is called at the end of each main-loop-cycle. It sets the id-variable
 *        of the vehicle such that the flashing pattern of the ID-LED can be chosen accordingly
 *        and sets the three positioning LEDs to be on permanently in normal-mode. In safe-mode
 *        it lets these three LEDs flash frequently and in test-mode it does not control these
 *        three LEDs.
 * \param vehicle_id_in
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void led_set_state(uint8_t vehicle_id_in);

/**
 * \brief This function is called each 20ms by an interrupt service routine and assures that
 *        the ID-LED of the vehicle flashes according to the vehicle's ID. In safe-mode it
 *        also lets the ID-LED flash like the other LEDs. In test-mode it does not control
 *        the ID-LED.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void led_toggle();

/**
 * \brief Configures the registers of the low_level_controller such that the LEDs can be controlled.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void led_setup();

/**
 * \brief In test-mode all leds are controlled via this function. It assigns the given values to the LEDs.
 * \param led1 positioning LED
 * \param led2 positioning LED
 * \param led3 positioning LED
 * \param led4 ID-LED
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void led_test(uint8_t led1, uint8_t led2, uint8_t led3, uint8_t led4);


#endif /* LED_H_ */