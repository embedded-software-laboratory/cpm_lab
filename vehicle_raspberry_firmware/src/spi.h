#pragma once
#include "../../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/spi_packets.h"

void spi_init();

void spi_transfer(
    spi_mosi_data_t spi_mosi_data,
    spi_miso_data_t *spi_miso_data_out,
    int *n_transmission_attempts_out,
    int *transmission_successful_out
);