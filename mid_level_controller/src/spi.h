#pragma once
#include "../../low_level_controller/vehicle_atmega2560_firmware/spi_packets.h"

/**
 * \brief Sets up all relevant registers such that SPI can be used.
 * \ingroup vehicle
 */
void spi_init();

/**
 * \brief Exchanges fixed bytes of data with the mid_level_controller via SPI.
 *        Note: This mid_level_controller is the master.
 * \ingroup vehicle
 * 
 * \param spi_mosi_data The package to be sent to low_level_controller.
 * \param spi_miso_data_out After transfer is finished, this variable contains the received
 *                          package from low_level_controller.
 * \param n_transmission_attempts_out After transfer is finished, this variable indicates the
 *                                    number of transmission attempts which were needed.
 * \param transmission_successful_out After transfer is finished, this flag indicates whether
 *                                    the transmission was successful (1) or not (0).
 */
void spi_transfer(
    spi_mosi_data_t spi_mosi_data,
    spi_miso_data_t *spi_miso_data_out,
    int *n_transmission_attempts_out,
    int *transmission_successful_out
);