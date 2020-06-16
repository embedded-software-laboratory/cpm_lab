#pragma once
#include "types.hpp"
#include "VehicleObservation.hpp"

class PoseCalculation
{
    std::vector<double> calibration_px;
    std::vector<double> calibration_py;
    std::vector<double> calibration_dx;
    std::vector<double> calibration_dy;

public:
    PoseCalculation();
    std::vector<VehicleObservation> apply(const VehiclePoints &vehiclePoints);
    
};