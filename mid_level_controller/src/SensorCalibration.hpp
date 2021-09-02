#pragma once


#include "VehicleState.hpp"

extern "C" {
#include "spi.h"
}

/**
 * \class SensorCalibration
 * \brief This class contains information on how raw sensor data of the low_level_controller
 *        can be interpreted.
 * \ingroup vehicle
 */
class SensorCalibration {

public:
    /**
     * \brief This function converts raw sensor data from low_level_controller into
     *        meaningful data, i.e. data with a common scale.
     * \param spi_miso_data The data package received from low_level_controller
     */
    static VehicleState convert(spi_miso_data_t spi_miso_data);
};