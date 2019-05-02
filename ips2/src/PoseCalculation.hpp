#pragma once
#include "types.hpp"
#include "VehicleObservation.hpp"

class PoseCalculation
{
public:
    PoseCalculation();
    std::vector<VehicleObservation> apply(const VehiclePoints &vehiclePoints);
    
};