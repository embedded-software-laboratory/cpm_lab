#pragma once

#include "VehicleCommandTrajectory.hpp"

/**
 * \struct TrajectoryInterpolation
 * \brief TODO
 * \ingroup vehicle
 */
struct TrajectoryInterpolation {
    //! TODO
    double t_now;
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
     * \param stamp_now TODO
     * \param start_point TODO
     * \param end_point TODO
     */
    TrajectoryInterpolation(uint64_t stamp_now, TrajectoryPoint start_point, TrajectoryPoint end_point);
};