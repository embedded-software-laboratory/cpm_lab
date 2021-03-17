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
 * \file adc.c
 *
 * \date Created: 26.09.2018 12:31:36
 * \author maczijewski
 * 
 * \ingroup low_level_controller
 */ 

#include "adc.h"
#include "util.h"
#include <util/delay.h>
#include <avr/io.h>


void adc_measure(uint16_t* battery_voltage, uint16_t* current_sense) {
	
	
	// battery measurement	
	SET_BIT(ADMUX, MUX0); // mux 100001 for ADC9
	_delay_us(125);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC) );
	*battery_voltage = ADC;	
	

	_delay_us(125);
	
	
	// current sense measurement	
	CLEAR_BIT(ADMUX, MUX0); // mux 100000 for ADC8
	_delay_us(125);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC) );
	*current_sense = ADC;
}


void adc_setup() {
	SET_BIT(ADCSRA, ADEN); // enable ADC
	SET_BIT(ADMUX, REFS1); // 1.1V reference
	SET_BIT(ADCSRB, MUX5); // mux[5:0] = 10000X for ADC8 or ADC9
	
	// set prescaler 128
	SET_BIT(ADCSRA, ADPS2);
	SET_BIT(ADCSRA, ADPS1);
	SET_BIT(ADCSRA, ADPS0);
}