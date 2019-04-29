#include "DetectVehicles.hpp"

DetectVehicles::DetectVehicles(VehiclePointSet &vehicle_point_set)
: vehicle_point_set(vehicle_point_set)
{

}

VehiclePoints DetectVehicles::apply(const FloorPoints &floor_points)
{
    VehiclePoints vehicle_points;
    return vehicle_points;
}
