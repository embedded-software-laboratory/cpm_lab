#pragma once

#include <stdint.h>

extern "C" {
#include "../../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/spi_packets.h"
}


class SimulationVehicle
{
    double x = 1;
    double y = 2;
    double distance = 0;
    double yaw = 4;
    double yaw_measured = 0;
    double speed = 0;
    double curvature = 0;

    
    uint32_t tick = 0;
    
    spi_mosi_data_t input_next; // save one input sample to simulate delay time


public:
    SimulationVehicle();
    spi_miso_data_t update(const spi_mosi_data_t spi_mosi_data, const double dt);
};