#pragma once

#include "VehicleCommandPathTracking.hpp"

/**
 * \struct PathInterpolation
 * \brief TODO
 * \ingroup vehicle
 */
struct PathInterpolation {
    //! TODO
    double s_queried;
    //! TODO
    double position_x;
    //! TODO
    double position_y;
    //! TODO
    double velocity_x;
    //! TODO
    double velocity_y;
    //! TODO
    double acceleration_x;
    //! TODO
    double acceleration_y;
    //! TODO
    double yaw;
    //! TODO
    double speed;
    //! TODO
    double curvature;

    /**
     * \brief TODO
     * \param s_queried TOOD
     * \param start_point TOOD
     * \param end_point TOOD
     */
    PathInterpolation(const double s_queried, const PathPoint start_point, const PathPoint end_point);
};