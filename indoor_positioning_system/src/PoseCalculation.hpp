#pragma once
#include "types.hpp"
#include "VehicleObservation.hpp"

/**
 * \class PoseCalculation
 * \brief TODO
 * \ingroup ips
 */
class PoseCalculation
{
    //! TODO
    std::vector<double> calibration_px;
    //! TODO
    std::vector<double> calibration_py;
    //! TODO
    std::vector<double> calibration_dx;
    //! TODO
    std::vector<double> calibration_dy;

public:
    /**
     * \brief Constructor TODO
     */
    PoseCalculation();

    /**
     * \brief TODO
     * \param vehiclePoints
     */
    std::vector<VehicleObservation> apply(const VehiclePoints &vehiclePoints);
    
};