#pragma once
#include "types.hpp"
#include "LedPoints.hpp"

class UndistortPoints
{


    std::vector<double> calibration_x;
    std::vector<double> calibration_y;


public:
    UndistortPoints(
        std::vector<double> _calibration_x, 
        std::vector<double> _calibration_y
    );
    FloorPoints apply(LedPoints led_points);
    
    
};