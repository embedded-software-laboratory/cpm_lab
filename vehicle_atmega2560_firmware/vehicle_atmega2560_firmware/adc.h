/*
 * adc.h
 *
 * Created: 26.09.2018 12:31:25
 *  Author: maczijewski
 */ 


#ifndef ADC_H_
#define ADC_H_


// The ADC is used to measure the battery voltage and the motor current sense.
// Battery: ADC9, use 1.1V reference, battery voltage is scaled appropriately with a voltage divider.
// Current Sense: ADC8, range: 0 mV to about 500 mV => use 1.1V reference

#include <stdint.h>

void adc_measure(uint16_t* battery_voltage, uint16_t* current_sense);
void adc_setup();


#endif /* ADC_H_ */