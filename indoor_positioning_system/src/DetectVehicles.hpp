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
#include <algorithm> // std::sort
#include <list>
#include <vector>
#include <array>
#include "types.hpp"
#include <opencv2/core.hpp>
#include "cpm/Logging.hpp"

/**
 * \class DetectVehicles
 * \brief TODO
 * \ingroup ips
 */
class DetectVehicles
{
public:
    /**
     * @brief Construct a new Detect Vehicles object
     * 
     * @param d_front_rear Distance between front and rear point
     * @param d_rear_rear Distance between rear points
     */
    DetectVehicles(const double &d_front_rear, const double &d_rear_rear);

    /**
     * @brief Search %FloorPoints and identify %VehiclePoints
     * 
     * @param floor_points 
     * @return VehiclePoints 
     */
    VehiclePoints apply(const FloorPoints &floor_points) const;

private:
    /** TODO: Move coordinate frame to appropriate file
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
     * 
     * From CAD 20190203 LEDholder:
     * back_left  = cv::Point2d(-0.07265, 0.017);
     * back_right = cv::Point2d(-0.07265, -0.017);
     * center     = cv::Point2d(0, 0);
     * front      = cv::Point2d(0.091, 0);
     * 
     */

    /**
     * @brief This factor times the actual point distance gives the tolerance 
     * 
     */
    const double point_distance_tolerance = 0.1;

    /**
     * @brief Tolerance [m] between front and rear point
     * 
     */
    const double tolerance_front_rear;

    /**
     * @brief Tolerance [m] between rear points
     * 
     */
    const double tolerance_rear_rear;
    
    /**
     * @brief Distance between front and rear point
     * 
     */
    const double d_front_rear;

    /**
     * @brief Distance between rear points
     * 
     */
    const double d_rear_rear;

    /**
     * @brief Calculate distances between point pairs and store in matrix
     * 
     * @param points 
     * @return cv::Mat_<double> 
     */
    cv::Mat_<double>
    calc_point_distances
    (
        const std::vector<cv::Point2d> &points
    ) const;

    /**
     * @brief Find point tripel satisfying the distances defined for a vehicle
     * 
     * @param points 
     * @param point_distances 
     * @return std::vector< std::array<std::size_t, 3> > 
     */
    std::vector< std::array<std::size_t, 4> >
    find_vehicle_candidates
    (
        const std::vector<cv::Point2d> &points,
        const cv::Mat_<double> &point_distances
    ) const;

    /**
     * @brief 
     * 
     * @param vehicle_candidates 
     * @return std::vector< std::array<std::size_t, 3> > 
     */
    std::vector< std::array<std::size_t, 4> >
    resolve_conflicts
    (
        const std::vector< std::array<std::size_t, 4> > &vehicle_candidates
    ) const;

    /**
     * @brief Determine conflicts between vehicle candidate pairs,
     * i.e. when a pair shares the same point
     * 
     * @param vehicle_candidates 
     * @return cv::Mat_<bool> 
     */
    cv::Mat_<bool>
    determine_conflicts
    (
        const std::vector< std::array<std::size_t, 4> > &vehicle_candidates
    ) const;

    /**
     * @brief Check if two arrays have an element in common
     * 
     * @param array_1 
     * @param array_2 
     * @return true 
     * @return false 
     */
    bool
    arrays_have_common_element
    (
        const std::array<std::size_t, 4> &array_1,
        const std::array<std::size_t, 4> &array_2
    ) const;

    /**
     * @brief Remove points assigned to vehicles from input floor_points
     * 
     * assumes no point is assigned to multiple vehicles
     * 
     * @param floor_points 
     * @param vehicle_candidates 
     * @return std::list<cv::Point2d> 
     */
    std::list<cv::Point2d>
    find_remaining_points
    (
        const std::vector<cv::Point2d> &floor_points,
        const std::vector< std::array<std::size_t, 4> > &vehicle_candidates
    ) const;

    /**
     * @brief Assign point tripel to a %VehiclePointSet
     * 
     * @param floor_points 
     * @param vehicle_candidate 
     * @return VehiclePointSet 
     */
    VehiclePointSet
    assign_vehicle_points
    (
        const std::vector<cv::Point2d> &floor_points,
        const std::array<std::size_t, 4> &vehicle_candidate
    ) const;

    /**
     * @brief Find center point and remove from remaining point list
     * 
     * @param vehicle_point_set 
     * @param remaining_points 
     */
    void find_center_point(VehiclePointSet &vehicle_point_set,
                           std::list<cv::Point2d> &remaining_points) const;  
};