#pragma once
#include "types.hpp"

class DetectVehicles
{
public:
    DetectVehicles(VehiclePointSet &vehicle_point_set);

    VehiclePoints apply(const FloorPoints &floor_points);

private:
    /**
     * @brief Point set defining LED geometry [m]
     * 
     * Point set defining the positions of the LEDs on a vehicle in [m]
     * in a coordinate frame with the origin in the center LED.
     * 
     * 
     *  back_left     
     *      o       y ^ 
     *                |
     *                o ---->      o  front
     *              center   x
     *      o
     *  back_right
     */
    VehiclePointSet vehicle_point_set;
    double point_distance_tolerance = 0.1;
};