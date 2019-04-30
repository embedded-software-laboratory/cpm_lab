#include "DetectVehicles.hpp"

DetectVehicles::DetectVehicles(const VehiclePointSet &vehicle_point_geometry)
: vehicle_point_geometry(vehicle_point_geometry)
{

}

VehiclePoints DetectVehicles::apply(const FloorPoints &floor_points)
{
    VehiclePoints vehicle_points;
    vehicle_points.timestamp = floor_points.timestamp;
    
    VehiclePointSet vehicle_point_set;
    vehicle_point_set.back_left  = cv::Point2d(0, 0);
    vehicle_point_set.back_right = cv::Point2d(0, 0);
    vehicle_point_set.center     = cv::Point2d(0, 0);
    vehicle_point_set.front      = cv::Point2d(0, 0);
    vehicle_points.vehicles.push_back(vehicle_point_set);
    return vehicle_points;
}
