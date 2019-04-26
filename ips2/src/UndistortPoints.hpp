#pragma once
#include "types.hpp"
#include "LedPoints.hpp"

class UndistortPoints
{
public:
    UndistortPoints();
    FloorPoints apply(LedPoints led_points);
    
    
};