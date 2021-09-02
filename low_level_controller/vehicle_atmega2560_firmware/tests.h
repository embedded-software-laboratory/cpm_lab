/**
 * \file tests.h
 *
 * \date 09/08/2019 09:48:25
 * \author maczijewski
 * 
 * \brief This module provides a basic test for relevant hardware components which
 *        can be enabled by using a jumper.
 * 
 * \ingroup low_level_controller
 */ 


#ifndef TESTS_H_
#define TESTS_H_

#include "spi_packets.h"
#include <avr/io.h>

/**
 * \brief Setup registers such that the tests can be enabled by using a jumper.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void tests_setup();

/**
 * \brief Performes technical tests.
 * 
 * It uses the odometer and the IMU to measure external movement applied to the vehicle.
 * Afterwards, it computes motor and servo control values which try to compensate the
 * external movement. In case that this test mode is active, no communication with the
 * mid_level_controller takes place.
 * For resulting behaviour see video at CPM Lab's confluence at
 * topic "ATmega Flashing" > "5. Tests".
 * 
 * \param packet_send Package containing all relevant sensor readings (in normal mode
 *                    sent to mid_level_controller)
 * \param packet_received Package constructed by this function and containing the control
 *                        values which try to compensate the external movement (in normal
 *                        mode received from mid_level_controller)
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void tests_apply(spi_miso_data_t *packet_send, spi_mosi_data_t *packet_received);

#endif /* TESTS_H_ */