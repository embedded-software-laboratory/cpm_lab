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

    // Converts the vehicle LED points from image coordinates to floor coordiantes
    FloorPoints apply(LedPoints led_points);
    
    
};