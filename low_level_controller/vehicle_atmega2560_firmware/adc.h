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
 * \file adc.h
 *
 * \date Created: 26.09.2018 12:31:25
 * \author maczijewski
 * 
 * \brief Provides an interface for measuring battery voltage and motor current.
 * 
 * \ingroup low_level_controller
 */ 


#ifndef ADC_H_
#define ADC_H_


#include <stdint.h>

/**
 * \brief After calling this function the given pointers will contain the results of the measurements.
 * \param battery_voltage The measured battery voltage
 * \param current_sense The measured motor current
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void adc_measure(uint16_t* battery_voltage, uint16_t* current_sense);

/**
 * \brief Setup of the AD-converter
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void adc_setup();


#endif /* ADC_H_ */