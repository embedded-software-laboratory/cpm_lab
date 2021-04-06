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

#include "DetectVehicles.hpp"

/**
 * \file DetectVehicles.cpp
 * \ingroup ips
 */

/**
 * \brief TODO
 * \param p TODO
 * \ingroup ips
 */
static inline double length(cv::Point2d p)
{
    return sqrt(p.dot(p));
}


DetectVehicles::DetectVehicles(const double &d_front_rear, const double &d_rear_rear)
: tolerance_front_rear(d_front_rear * point_distance_tolerance),
  tolerance_rear_rear(d_rear_rear * point_distance_tolerance),
  d_front_rear(d_front_rear),
  d_rear_rear(d_rear_rear)
{
}


VehiclePoints DetectVehicles::apply(const FloorPoints &floor_points) const
{
    // calulate distances between each point pair and save in matrix
    const cv::Mat_<double> point_distances = 
        calc_point_distances(floor_points.points);
    // find point tripel resembling a vehicle point set
    std::vector< std::array<std::size_t, 4> > vehicle_candidates = 
        find_vehicle_candidates(floor_points.points,
                                point_distances);
    // filter candidates so that every point is uniquely assigned to one vehicle
    std::vector< std::array<std::size_t, 4> > vehicles =
        resolve_conflicts(vehicle_candidates);

    VehiclePoints vehicle_points;
    vehicle_points.timestamp = floor_points.timestamp;
    for (const std::array<std::size_t, 4> &veh : vehicles)
    {
        // assign points to vehicle point set
        VehiclePointSet vehicle_point_set = 
            assign_vehicle_points(floor_points.points,
                                  veh);
        // add vehicle point set to vehicle_points
        vehicle_points.vehicles.push_back(vehicle_point_set);
    }
    return vehicle_points;
}


cv::Mat_<double> DetectVehicles::calc_point_distances
(
    const std::vector<cv::Point2d> &points
) const
{
    cv::Mat_<double> point_distances = cv::Mat_<double>::zeros(points.size(), points.size());
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        for (std::size_t j = i+1; j < points.size(); ++j)
        {
            point_distances(i, j) = length(points[i] - points[j]);
            point_distances(j, i) = point_distances(i, j);
        }            
    }
    return point_distances;    
}


std::vector< std::array<std::size_t, 4> > 
DetectVehicles::find_vehicle_candidates
(
    const std::vector<cv::Point2d> &points,
    const cv::Mat_<double> &point_distances
) const
{
    std::vector< std::array<std::size_t, 4> > vehicle_candidates;
    // 3 points are necessary to consitute a vehicle
    if (points.size() < 3) return {};

    // iterate through all index combinations
    for (std::size_t i = 0; i < points.size()-2; ++i)
    {
        for (std::size_t j = i+1; j < points.size()-1; ++j)
        {
            for (std::size_t k = j+1; k < points.size(); ++k)
            {
                
                int num_rear_rear = 0;
                int num_front_rear = 0;
                std::array<double, 3> distances = {point_distances(i, j),
                                                point_distances(i, k),
                                                point_distances(j, k)};
                // check distances
                for (const double &d : distances)
                {
                    // points in front-back distance?
                    if (std::fabs(d - d_front_rear) < tolerance_front_rear)
                    {
                        ++num_front_rear;
                    }
                    // points in back-back distance?
                    else if (std::fabs(d - d_rear_rear) < tolerance_rear_rear)
                    {
                        ++num_rear_rear;
                    }
                    
                }
                // one rear pair and two front-rear pairs are required for a vehicle
                if (num_rear_rear == 1 && num_front_rear == 2)
                {
                    std::array<std::size_t, 4> vehicle_candidate = {i, j, k, ULONG_MAX};

                    // Check for ID LED 
                    // weighted average: 70/152.5 for front point, 82.5/152.5/2 for rear points
                    VehiclePointSet vehicle_point_set = assign_vehicle_points(points, vehicle_candidate);  
                    cv::Point2d center_point = 0.270491803 * (vehicle_point_set.back_left + vehicle_point_set.back_right)
                                                + 0.459016393 * vehicle_point_set.front;
                        

                    // is front point in correct angle to vehicle back? 
                    double mod1   = sqrt(pow(vehicle_point_set.back_left.y-vehicle_point_set.front.y,2)+pow(vehicle_point_set.back_left.x-vehicle_point_set.front.x,2));
                    double mod2   = sqrt(pow(vehicle_point_set.back_right.y-vehicle_point_set.front.y,2)+pow(vehicle_point_set.back_right.x-vehicle_point_set.front.x,2));
                    double innerp = (vehicle_point_set.back_left.x-vehicle_point_set.front.x)*(vehicle_point_set.back_right.x-vehicle_point_set.front.x) + (vehicle_point_set.back_left.y-vehicle_point_set.front.y)*(vehicle_point_set.back_right.y-vehicle_point_set.front.y);
                    double angle  = acos(innerp / (mod1 * mod2)) * 180 / M_PI;

                    if (!(angle-0.9 <= 11.8613 && angle+0.9 >= 11.8613))
                    {
                        // incorrect orientation - front point does not match to vehicle back
                        continue; 
                    }


                    // add ID LED
                    for (std::size_t l = 0; l < points.size(); l++) 
                    {
                        if (fabs(length(center_point - points[l])) < 0.03) 
                        {
                            // correct distances  
                            vehicle_candidate = {i,j,k,l}; 
                            break;
                        }
                    }

                    vehicle_candidates.push_back(vehicle_candidate);
                }
            }
        }
    }
    return vehicle_candidates;
}


std::vector< std::array<std::size_t, 4> >
DetectVehicles::resolve_conflicts
(
    const std::vector< std::array<std::size_t, 4> > &vehicle_candidates
) const
{
    // Matrix stores conflicts between vehicle candidates, i.e. when a point is used by both
    cv::Mat_<int> mat_conflicts = determine_conflicts(vehicle_candidates);
    // if there is one or less vehicle candidates, there are no conflicts
    if (mat_conflicts.total() <= 1) return vehicle_candidates;

    std::vector< std::array<std::size_t, 4> > vehicles;
    // vector indicating that a vehicle candidate is resolved
    std::vector<bool> resolved;
    resolved.insert(resolved.begin(), vehicle_candidates.size(), 0);

    // loop while conflicts can be further resolved
    bool is_conflict_solvable = 1;
    while (is_conflict_solvable)
    {
        is_conflict_solvable = 0;
        for (std::size_t m = 0; m < mat_conflicts.col(0).total(); ++m)
        {
            if (resolved[m]) 
            {
                continue;
            }
            
            // sum conflicts
            std::vector<std::size_t> conflicts;
            for (std::size_t n = 0; n < mat_conflicts.row(0).total(); ++n)
            {
                if (mat_conflicts(m, n))
                {
                    conflicts.push_back(n);
                }
            }
            // no conflicts
            if (conflicts.size() == 0)
            {
                resolved[m] = 1;
                // add vehicle candidate to vehicles
                vehicles.push_back(vehicle_candidates[m]);
                continue;
            }
            // one conflict
            else if (conflicts.size() == 1)
            {
                // if other vehicle has more conflicts,
                // accept the current vehicle
                std::size_t other_veh_conflicts
                    = cv::countNonZero(mat_conflicts.row(conflicts[0]));
                if (other_veh_conflicts > conflicts.size())
                {
                    // add vehicle candidate to vehicles
                    vehicles.push_back(vehicle_candidates[m]);
                    resolved[m] = 1;
                    mat_conflicts.col(m) = 0;
                    // mark conflict vehicle as resolved
                    resolved[conflicts[0]] = 1;
                    mat_conflicts.col(conflicts[0]) = 0;
                    // change has occurred, so mark conflicts as solvable
                    is_conflict_solvable = 1;
                }                
            }
            
        }
    }

    //all conflicts resolved?
    for (std::size_t m = 0; m < mat_conflicts.col(0).total(); ++m)
    {
        if(!resolved[m])
        {
            cpm::Logging::Instance().write(
                1,
                "%s",
                "IPS conflict not solvable"
            );
            break;
        }
    }

    return vehicles;
}


cv::Mat_<bool> DetectVehicles::determine_conflicts
(
    const std::vector< std::array<std::size_t, 4> > &vehicle_candidates
) const
{
    cv::Mat_<bool> mat_conflicts = cv::Mat_<bool>::zeros(vehicle_candidates.size(),
                                                       vehicle_candidates.size());
    
    if (vehicle_candidates.size() <= 1) return mat_conflicts;
    // loop through all vehicle candidate combinations
    for (std::size_t i = 0; i < vehicle_candidates.size()-1; ++i)
    {
        for (std::size_t j = i+1; j < vehicle_candidates.size(); ++j)
        {
            // check if vehicle combination shares led point
            if (arrays_have_common_element(vehicle_candidates[i], vehicle_candidates[j]))
            {
                mat_conflicts(i, j) = 1;
                mat_conflicts(j, i) = 1;
            }
        }
    }
    return mat_conflicts;
}

bool DetectVehicles::arrays_have_common_element
(
    const std::array<std::size_t, 4> &array_1,
    const std::array<std::size_t, 4> &array_2
) const
{
    for (const std::size_t &i : array_1)
    {
        // ULONG_MAX if no ID LED is present - no conflict 
        if (i == ULONG_MAX) continue;
        
        for (const std::size_t &j : array_2)
        {
            if (i == j)
            {
                return true;
            }
        }
    }
    return false;
}

VehiclePointSet DetectVehicles::assign_vehicle_points
(
    const std::vector<cv::Point2d> &floor_points,
    const std::array<std::size_t, 4> &vehicle_candidate
) const
{
    // find rear points
    std::vector<std::size_t> rear_edge_pair;
    // ignore ID LED 
    for (std::size_t i = 0; i < vehicle_candidate.size() - 2; ++i)
    {
        for (std::size_t j = i + 1; j < vehicle_candidate.size() -1; ++j)
        {
            cv::Point2d point1 = floor_points[vehicle_candidate[i]];
            cv::Point2d point2 = floor_points[vehicle_candidate[j]];

            if (fabs(length(point1-point2) - d_rear_rear) < tolerance_rear_rear)
            {
                rear_edge_pair.push_back(vehicle_candidate[i]);
                rear_edge_pair.push_back(vehicle_candidate[j]);
            }
        }
    }
    assert(rear_edge_pair.size() == 2);

    // determine front point
    cv::Point2d front;
    for (const std::size_t i : vehicle_candidate)
    {
        // ignore ID LED 
        if (i != rear_edge_pair[0] && i != rear_edge_pair[1] && i != vehicle_candidate[3])
        {
            front = floor_points[i];
        }
    }

    // determine rear left and right
    const auto rear0 = floor_points[rear_edge_pair[0]];
    const auto rear1 = floor_points[rear_edge_pair[1]];
    cv::Point2d rear_left;
    cv::Point2d rear_right;

    if((rear0 - front).cross(rear1 - front) > 0)
    {
        rear_left = rear0;
        rear_right = rear1;
    }
    else 
    {
        rear_left = rear1;
        rear_right = rear0;
    }

    VehiclePointSet vehicle_point_set;
    vehicle_point_set.front = front;
    vehicle_point_set.back_left = rear_left;
    vehicle_point_set.back_right = rear_right;    

    // determine ID LED
    if (vehicle_candidate[3] == ULONG_MAX) 
    {
        // no ID LED
        vehicle_point_set.center_present = 0; 
    }
    else
    {
        // ID LED 
        vehicle_point_set.center_present = 1;
        vehicle_point_set.center = floor_points[vehicle_candidate[3]];
    }
    

    return vehicle_point_set;
}