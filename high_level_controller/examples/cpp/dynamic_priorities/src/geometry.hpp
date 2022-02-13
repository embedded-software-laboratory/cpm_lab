// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

#pragma once
#include <vector>
#include <cassert>
#include <cmath>
using std::vector;

const float VEHICLE_HALF_LENGTH = 0.15;
const float VEHICLE_HALF_WIDTH = 0.06;

/**
 * \file geometry.hpp
 * Defines helper functions for calculating distances between vehicles
 */

/**
 * \struct PathNode
 * \ingroup distributed_routing
 */
struct PathNode
{
    //! x-position, meters
    double x;
    //! y-position, meters
    double y;
    //! Cosine of angle of pose
    double cos_yaw;
    //! Sine of angle of pose
    double sin_yaw;
    
    //! Creates a PathNode
    PathNode(){}

    /**
     * \brief Creates a PathNode
     * \param x x-position, meters
     * \param y y-position, meters
     * \param cos_yaw Cosine of angle of pose
     * \param sin_yaw Sine of angle of pose
     */
    PathNode(double x, double y, double cos_yaw, double sin_yaw)
    :x(x), y(y), cos_yaw(cos_yaw), sin_yaw(sin_yaw){}
};

/**
 * \brief Calculates the shortest distance between edges of a vehicle and a point
 * \ingroup decentral_routing
 * \param vehicle Pose of the vehicle, as a PathNode
 * \param points_x x-coordinate of point
 * \param points_y y-coordinate of point
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
 * \brief Calculates the shortest distance between 2 vehicles
 * \ingroup decentral_routing
 * \param vehicleA Pose of vehicle A as a PathNode
 * \param vehicleB Pose of vehicle B as a PathNode
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
