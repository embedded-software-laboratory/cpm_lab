#pragma once
#include "types.hpp"
#include "UndistortPoints.hpp"
#include "LedPoints.hpp"
#include "DetectVehicles.hpp"
#include "DetectVehicleID.hpp"
#include "PoseCalculation.hpp"
#include <memory>

class IpsPipeline
{

    std::shared_ptr<UndistortPoints> undistortPointsFn;
    std::shared_ptr<DetectVehicles> detectVehiclesFn;
    std::shared_ptr<DetectVehicleID> detectVehicleIDfn;
    std::shared_ptr<PoseCalculation> poseCalculationFn;

    VehiclePointTimeseries vehiclePointTimeseries;

public:
    IpsPipeline();
    void apply(LedPoints led_points);
    
    
};