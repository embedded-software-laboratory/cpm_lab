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