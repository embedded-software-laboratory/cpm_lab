#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/spi_packets.h"

void spi_atmega_setup();

spi_miso_data_t spi_atmega_exchange(spi_mosi_data_t send_data);