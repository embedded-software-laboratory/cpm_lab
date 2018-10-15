#include "Localization.hpp"



Pose2D Localization::sensor_update(VehicleState vehicleState) {
    const double imu_yaw = vehicleState.pose().yaw();

    // Calculate continuous yaw angle (with number of rotations)
    double delta_yaw = imu_yaw - imu_yaw_previous;
    if(delta_yaw > M_PI) delta_yaw -= 2*M_PI;
    else if(delta_yaw < -M_PI) delta_yaw += 2*M_PI;
    pose2D.yaw(pose2D.yaw() + delta_yaw);


    // Dead reckoning
    const double odometer_distance = vehicleState.odometer_distance();    
    const double ds = odometer_distance - odometer_distance_previous;
    if(-0.5 < ds && ds < 0.5) {
        pose2D.x(pose2D.x() + ds * cos(pose2D.yaw()));
        pose2D.y(pose2D.y() + ds * sin(pose2D.yaw()));
    }

    odometer_distance_previous = odometer_distance;
    imu_yaw_previous = imu_yaw;

    return pose2D;
}