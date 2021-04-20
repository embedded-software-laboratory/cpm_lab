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
 * \file watchdog.h
 *
 * \date Created: 6/21/2019 13:35:35
 * \author: cfrauzem
 * 
 * \brief The watchdog of the low_level_controller, which can be accessed via this module,
 *        controls the state of the vehicle. If the watchdog is enabled and the
 *        low_level_controller didn't get up-to-date information of the mid_level_controller
 *        for too long, the watchdog will provoke the low_level_controller to be set into
 *        safe-mode.
 * 
 * \ingroup low_level_controller
 */ 

#ifndef WATCHDOG_H_
#define WATCHDOG_H_


#include "spi_packets.h"

extern volatile uint8_t safe_mode_flag;

/**
 * \brief Disable the functionality of the watchdog.
 *
 * \author cfrauzem
 * \ingroup low_level_controller
 */ 
void watchdog_disable();

/**
 * \brief Enable the functionality of the watchdog.
 *
 * \author cfrauzem
 * \ingroup low_level_controller
 */ 
void watchdog_enable();

/**
 * \brief Reset the watchdog to the initial value. Has to be done regularily such that
 *        the safe-mode is not provoked when low_level_controller and mid_level_controller
 *        work together in a proper way.
 *
 * \author cfrauzem
 * \ingroup low_level_controller
 */ 
void watchdog_reset();

#endif /* WATCHDOG_H_ */