#pragma once
#include <cstdint>
#include <vector>
#include "types.hpp"

class DetectVehicleID
{
    const std::vector<uint8_t> identification_LED_period_ticks;
    const std::vector<uint8_t> identification_LED_enabled_ticks;

public:
    DetectVehicleID(
        std::vector<uint8_t> _identification_LED_period_ticks,
        std::vector<uint8_t> _identification_LED_enabled_ticks
    );
    VehiclePoints apply(VehiclePointTimeseries vehiclePointTimeseries);
    
};