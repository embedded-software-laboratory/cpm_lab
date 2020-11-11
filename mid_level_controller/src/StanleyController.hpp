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
// #include <string>
// #include <array>
// #include "VehicleModel.hpp"
#include "VehicleCommandPathTracking.hpp"
#include "VehicleState.hpp"
// #include "Visualization.hpp"
// #include "cpm/get_topic.hpp"
// #include <dds/pub/ddspub.hpp>


class StanleyController
{
    uint8_t vehicle_id;

    Pose2D find_reference_pose2d(
        const std::vector<PathPoint> &path,
        const double x,
        const double y,
        const double yaw
    );

    std::function<void(double&, double&)> stop_vehicle;


public:

    StanleyController(uint8_t vehicle_id, std::function<void(double&, double&)> stop_vehicle);

    double control_steering_servo(
        const VehicleState &vehicleState,                  
        const VehicleCommandPathTracking &commandPathTracking
    );

    void reset_ref_path();
    
};