#pragma once
#include "../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/spi_packets.h"

void spi_init();
spi_miso_data_t spi_transfer(spi_mosi_data_t spi_mosi_data);