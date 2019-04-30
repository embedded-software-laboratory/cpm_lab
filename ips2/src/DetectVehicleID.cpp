#include "DetectVehicleID.hpp"


DetectVehicleID::DetectVehicleID(
    std::vector<uint8_t> _identification_LED_period_ticks,
    std::vector<uint8_t> _identification_LED_enabled_ticks
)
:identification_LED_period_ticks(_identification_LED_period_ticks)
,identification_LED_enabled_ticks(_identification_LED_enabled_ticks)
{

}


VehiclePoints DetectVehicleID::apply(VehiclePointTimeseries vehiclePointTimeseries)
{
    VehiclePoints result;
    return result;
}

