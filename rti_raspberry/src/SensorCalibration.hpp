#pragma once


#include "VehicleState.hpp"

extern "C" {
#include "spi.h"
}

class SensorCalibration {

public:
    static VehicleState convert(spi_miso_data_t spi_miso_data);
};