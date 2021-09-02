#pragma once
#include <vector>
#include <cassert>
#include <cmath>
using std::vector;

/**
 * \file geometry.hpp
 * \ingroup vehicle
 * \brief (Needs to be included for static functions to show up properly)
 */

/**
 * \brief TODO
 * \ingroup vehicle
 */
#define VEHICLE_HALF_LENGTH (0.11)

/**
 * \brief TODO
 * \ingroup vehicle
 */
#define VEHICLE_HALF_WIDTH (0.054)

/**
 * \struct PathNode
 * \brief TODO
 * \ingroup vehicle
 */
struct PathNode
{
    //! TODO
    double x;
    //! TODO
    double y;
    //! TODO
    double cos_yaw;
    //! TODO
    double sin_yaw;

    /**
     * \brief TODO Constructor
     */
    PathNode(){}

    /**
     * \brief TODO Constructor
     * \param x TODO
     * \param y TODO
     * \param cos_yaw TODO
     * \param sin_yaw TODO
     */
    PathNode(double x, double y, double cos_yaw, double sin_yaw)
    :x(x), y(y), cos_yaw(cos_yaw), sin_yaw(sin_yaw){}
};

/**
 * \brief TODO
 * \ingroup vehicle
 * 
 * \param vehicle TODO
 * \param points_x TODO
 * \param points_y TODO
 */
static inline double min_distance_vehicle_to_points
(
    PathNode vehicle,
    const vector<double> &points_x,
    const vector<double> &points_y
)
{
    assert(points_x.size() == points_y.size());
    double min_distance = 1e300;

    for (size_t i = 0; i < points_x.size(); ++i)
    {
        const double dx = vehicle.x - points_x[i];
        const double dy = vehicle.y - points_y[i];

        const double dist_longitudinal = fmax(fabs(dx * vehicle.cos_yaw + dy * vehicle.sin_yaw) - VEHICLE_HALF_LENGTH, 0);
        const double dist_lateral      = fmax(fabs(dy * vehicle.cos_yaw - dx * vehicle.sin_yaw) - VEHICLE_HALF_WIDTH, 0);

        min_distance = fmin(min_distance, fmax(dist_longitudinal, dist_lateral));
    }

    return min_distance;
}

/**
 * \brief TODO
 * \ingroup vehicle
 * 
 * \param vehicleA TODO
 * \param vehicleB TODO
 */
static inline double min_distance_vehicle_to_vehicle(PathNode vehicleA, PathNode vehicleB)
{
    const vector<double> cornersA_x
    {
        vehicleA.x + vehicleA.cos_yaw * VEHICLE_HALF_LENGTH + vehicleA.sin_yaw * VEHICLE_HALF_WIDTH,
        vehicleA.x + vehicleA.cos_yaw * VEHICLE_HALF_LENGTH - vehicleA.sin_yaw * VEHICLE_HALF_WIDTH,
        vehicleA.x - vehicleA.cos_yaw * VEHICLE_HALF_LENGTH + vehicleA.sin_yaw * VEHICLE_HALF_WIDTH,
        vehicleA.x - vehicleA.cos_yaw * VEHICLE_HALF_LENGTH - vehicleA.sin_yaw * VEHICLE_HALF_WIDTH
    };

    const vector<double> cornersA_y
    {
        vehicleA.y + vehicleA.sin_yaw * VEHICLE_HALF_LENGTH - vehicleA.cos_yaw * VEHICLE_HALF_WIDTH,
        vehicleA.y + vehicleA.sin_yaw * VEHICLE_HALF_LENGTH + vehicleA.cos_yaw * VEHICLE_HALF_WIDTH,
        vehicleA.y - vehicleA.sin_yaw * VEHICLE_HALF_LENGTH - vehicleA.cos_yaw * VEHICLE_HALF_WIDTH,
        vehicleA.y - vehicleA.sin_yaw * VEHICLE_HALF_LENGTH + vehicleA.cos_yaw * VEHICLE_HALF_WIDTH
    };

    const vector<double> cornersB_x
    {
        vehicleB.x + vehicleB.cos_yaw * VEHICLE_HALF_LENGTH + vehicleB.sin_yaw * VEHICLE_HALF_WIDTH,
        vehicleB.x + vehicleB.cos_yaw * VEHICLE_HALF_LENGTH - vehicleB.sin_yaw * VEHICLE_HALF_WIDTH,
        vehicleB.x - vehicleB.cos_yaw * VEHICLE_HALF_LENGTH + vehicleB.sin_yaw * VEHICLE_HALF_WIDTH,
        vehicleB.x - vehicleB.cos_yaw * VEHICLE_HALF_LENGTH - vehicleB.sin_yaw * VEHICLE_HALF_WIDTH
    };

    const vector<double> cornersB_y
    {
        vehicleB.y + vehicleB.sin_yaw * VEHICLE_HALF_LENGTH - vehicleB.cos_yaw * VEHICLE_HALF_WIDTH,
        vehicleB.y + vehicleB.sin_yaw * VEHICLE_HALF_LENGTH + vehicleB.cos_yaw * VEHICLE_HALF_WIDTH,
        vehicleB.y - vehicleB.sin_yaw * VEHICLE_HALF_LENGTH - vehicleB.cos_yaw * VEHICLE_HALF_WIDTH,
        vehicleB.y - vehicleB.sin_yaw * VEHICLE_HALF_LENGTH + vehicleB.cos_yaw * VEHICLE_HALF_WIDTH
    };


    const double min_dist_A = min_distance_vehicle_to_points(vehicleA, cornersB_x, cornersB_y);
    const double min_dist_B = min_distance_vehicle_to_points(vehicleB, cornersA_x, cornersA_y);

    return fmin(min_dist_A, min_dist_B);
}