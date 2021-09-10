#pragma once
#include "types.hpp"
#include "LedPoints.hpp"

/**
 * \class UndistortPoints
 * \brief TODO
 * \ingroup ips
 */
class UndistortPoints
{

    //! TODO
    std::vector<double> calibration_x;
    //! TODO
    std::vector<double> calibration_y;


public:
    /**
     * \brief Constructor TODO
     * \param _calibration_x TODO
     * \param _calibration_y TODO
     */
    UndistortPoints(
        std::vector<double> _calibration_x, 
        std::vector<double> _calibration_y
    );

    /**
     * \brief Converts the vehicle LED points from image coordinates to floor coordiantes
     * \param led_points TODO
     */
    FloorPoints apply(LedPoints led_points);
    
    
};