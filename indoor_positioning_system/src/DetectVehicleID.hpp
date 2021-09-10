#pragma once
#include <cstdint>
#include <vector>
#include "cpm/Logging.hpp"
#include "types.hpp"

/**
 * \class DetectVehicleID
 * \brief TODO
 * \ingroup ips
 */
class DetectVehicleID
{
    //! TODO
    const std::vector<uint8_t> identification_LED_period_ticks;
    //! TODO
    const std::vector<uint8_t> identification_LED_enabled_ticks;

public:
    /**
     * \brief Constructor TODO
     * \param _identification_LED_period_ticks TODO
     * \param _identification_LED_enabled_ticks TODO
     */
    DetectVehicleID(
        std::vector<uint8_t> _identification_LED_period_ticks,
        std::vector<uint8_t> _identification_LED_enabled_ticks
    );


    /**
     * \brief Tracks vehicles over time and identifies ID from middle LED
     * \param vehiclePointTimeseries List of VehiclePoints. Time between entries is required to be 20 ms
     * \return VehiclePoints with correct ID
     */
    VehiclePoints apply(const VehiclePointTimeseries &vehiclePointTimeseries);
    
};