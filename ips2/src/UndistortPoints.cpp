#include "UndistortPoints.hpp"
#include <cassert>


#define N_CALIBRATION_TERMS (15)

UndistortPoints::UndistortPoints(
    std::vector<double> _calibration_x, 
    std::vector<double> _calibration_y
)
:calibration_x(_calibration_x)
,calibration_y(_calibration_y)
{
    assert(calibration_x.size() == N_CALIBRATION_TERMS);
    assert(calibration_y.size() == N_CALIBRATION_TERMS);
}


FloorPoints UndistortPoints::apply(LedPoints led_points)
{
    FloorPoints result;
    result.timestamp = led_points.time_stamp().nanoseconds();

    for(auto image_point:led_points.led_points())
    {
        double image_x = image_point.x() / 2048.0;
        double image_y = image_point.y() / 2048.0;

        double ix1 = image_x;
        double ix2 = ix1 * image_x;
        double ix3 = ix2 * image_x;
        double ix4 = ix3 * image_x;

        double iy1 = image_y;
        double iy2 = iy1 * image_y;
        double iy3 = iy2 * image_y;
        double iy4 = iy3 * image_y;

        // Calibration based on two dimensional, 4th order polynomial
        double features[] = {
            1, 
            ix1, iy1, 
            ix2, ix1 * iy1, iy2, 
            ix3, ix2 * iy1, ix1 * iy2, iy3,
            ix4, ix3 * iy1, ix2 * iy2, ix1 * iy3, iy4
        };

        double floor_x = 0;
        double floor_y = 0;
        for (int i = 0; i < N_CALIBRATION_TERMS; ++i)
        {
            floor_x += features[i] * calibration_x[i];
            floor_y += features[i] * calibration_y[i];
        }
        result.points.emplace_back(floor_x, floor_y);
    }

    return result;
}
    
