/**
 * \file spi.h
 *
 * \date Created: 21.09.2018 19:42:56
 * \author maczijewski
 * 
 * \brief In order to make communication between mid_level_controller and
 *        low_level_controller possible the Serial Peripheral Interface (SPI)
 *        is used. This module here provides an interface to use SPI for
 *        communicating with the mid_level_controller.
 * 
 * \ingroup low_level_controller
 */ 


#ifndef SPI_H_
#define SPI_H_


#include "spi_packets.h"

/**
 * \brief Exchanges fixed bytes of data with the mid_level_controller via SPI.
 *        Note: This low_level_controller is the slave. Since the communication
 *              is synchronous, spi_exchange will block until the transfer is
 *              completed.
 * \param packet_send       The package to be sent to mid_level_controller.
 * \param packet_received   The received package from mid_level_controller.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void spi_exchange(spi_miso_data_t *packet_send, spi_mosi_data_t *packet_received);

/**
 * \brief Sets up all relevant registers such that SPI can be used.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void spi_setup();


#endif /* SPI_H_ */