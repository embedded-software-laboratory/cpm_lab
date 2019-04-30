#pragma once
#include "types.hpp"
#include "UndistortPoints.hpp"
#include "LedPoints.hpp"
#include "DetectVehicles.hpp"
#include <memory>

class IpsPipeline
{

    std::shared_ptr<UndistortPoints> undistortPointsFn;
    std::shared_ptr<DetectVehicles> detectVehiclesFn;

public:
    IpsPipeline();
    void apply(LedPoints led_points);
    
    
};