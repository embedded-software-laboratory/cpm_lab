#pragma once
#include "types.hpp"
#include "UndistortPoints.hpp"
#include "LedPoints.hpp"
#include <memory>

class IpsPipeline
{

    std::shared_ptr<UndistortPoints> undistortPointsFn;

public:
    IpsPipeline();
    void apply(LedPoints led_points);
    
    
};