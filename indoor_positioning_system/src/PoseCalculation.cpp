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

#include "PoseCalculation.hpp"
#include <cassert>

#define N_POSE_CALIBRATION_TERMS (5)


PoseCalculation::PoseCalculation()
:calibration_px({1.0033,     0.0,   0.0792,      0.0,     0.0})
,calibration_py({   0.0,  1.0033,      0.0,   0.0792,     0.0})
,calibration_dx({   0.0,     0.0,      1.0,   0.0038,     0.0})
,calibration_dy({   0.0,     0.0,  -0.0054,      1.0,     0.0})
{
    assert(calibration_px.size() == N_POSE_CALIBRATION_TERMS);
    assert(calibration_py.size() == N_POSE_CALIBRATION_TERMS);
    assert(calibration_dx.size() == N_POSE_CALIBRATION_TERMS);
    assert(calibration_dy.size() == N_POSE_CALIBRATION_TERMS);
}


static inline double length(cv::Point2d p)
{
    return sqrt(p.dot(p));
}

std::vector<VehicleObservation> PoseCalculation::apply(const VehiclePoints &vehiclePoints)
{
    std::vector<VehicleObservation> result;

    for(const auto &vehicle : vehiclePoints.vehicles)
    {
        const cv::Point2d back = 0.5 * (vehicle.back_left + vehicle.back_right);
        const cv::Point2d front = vehicle.front;
        const cv::Point2d direction = front - back;
        const cv::Point2d direction_normalized = direction * (1.0 / length(direction));

        const double features[] = {
            back.x, back.y, direction_normalized.x, direction_normalized.y, 1
        };


        // Uncomment this when creating a new pose calibration
        /*
        std::cout << "pose_features ";
        for (int i = 0; i < N_POSE_CALIBRATION_TERMS; ++i) std::cout << features[i] << ",  ";
        std::cout << std::endl;
        */

    

        double ref_position_x = 0;
        double ref_position_y = 0;
        double ref_direction_x = 0;
        double ref_direction_y = 0;
        for (int i = 0; i < N_POSE_CALIBRATION_TERMS; ++i)
        {
            ref_position_x += features[i] * calibration_px[i];
            ref_position_y += features[i] * calibration_py[i];
            ref_direction_x += features[i] * calibration_dx[i];
            ref_direction_y += features[i] * calibration_dy[i];

        }

        VehicleObservation vehicleObservation;
        vehicleObservation.header().create_stamp().nanoseconds(vehiclePoints.timestamp);
        vehicleObservation.header().valid_after_stamp().nanoseconds(vehiclePoints.timestamp);
        vehicleObservation.pose().x(ref_position_x);
        vehicleObservation.pose().y(ref_position_y);
        vehicleObservation.pose().yaw(atan2(ref_direction_y, ref_direction_x));
        vehicleObservation.vehicle_id(vehicle.id);
        result.push_back(vehicleObservation);
    }


    return result;
}