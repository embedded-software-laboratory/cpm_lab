/**
 * \file odometer.h
 *
 * \date 20.09.2018 17:43:13
 * \author: maczijewski
 * 
 * \brief This module provides an interface for setting up and reading values of the odometer and inferred values.
 * 
 * For implementation details see \link odometer.c \endlink.
 */ 


#ifndef ODOMETER_H_
#define ODOMETER_H_

/**
 * \brief Current speed of the vehicle. Inferred by using the last few odometer steps and
 *        the time between these subsequent measurements.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
int16_t get_speed();

/**
 * \brief Returns the current count of odometer steps. Note: Overflow after about 6705km ;)
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
int32_t get_odometer_count();

/**
 * \brief Sets up all relevant registers such that the odometer provides correct values.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void odometer_setup();


#endif /* ODOMETER_H_ */