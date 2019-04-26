#pragma once
#include "types.hpp"
#include "UndistortPoints.hpp"
#include "LedPoints.hpp"

class IpsPipeline
{
public:
    IpsPipeline();
    void apply(LedPoints led_points);
    
    
};