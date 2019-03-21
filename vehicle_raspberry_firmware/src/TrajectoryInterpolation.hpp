#pragma once

#include "VehicleCommandTrajectory.hpp"

struct TrajectoryInterpolation {

    double t_now;
    double position_x;
    double position_y;
    double velocity_x;
    double velocity_y;
    double acceleration_x;
    double acceleration_y;
    double yaw;
    double speed;
    double curvature;

    TrajectoryInterpolation(uint64_t stamp_now, TrajectoryPoint start_point, TrajectoryPoint end_point);
};