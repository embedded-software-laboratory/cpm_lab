#pragma once

#include "VehicleState.hpp"

class Localization
{
    double imu_yaw_previous = 0;
    double odometer_distance_previous = 0;
    Pose2D pose2D = Pose2D(0,0,0);
public:
    Pose2D sensor_update(VehicleState vehicleState);
    void reset() {pose2D = Pose2D(0,0,0);}
    // TODO add IPS observation update
    
};