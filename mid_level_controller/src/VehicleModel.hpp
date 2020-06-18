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
#include <cmath>

class VehicleModel
{
    public:
        static void step(
            const std::vector<double> &dynamics_parameters,
            const double dt,
            const double motor_throttle,
            const double steering_servo,
            const double battery_voltage,
            double &px, double &py, double &yaw, double &speed);

        static void step(
            const std::vector<double> &dynamics_parameters,
            const double dt,
            const double motor_throttle,
            const double steering_servo,
            const double battery_voltage,
            double &px, double &py, double &yaw, double &speed,
            double &d_px, double &d_py, double &d_yaw, double &d_speed);

    private:
        static void log_sim_warnings(
            const double motor_throttle,
            const double steering_servo,
            const double speed,
            const double d_speed,
            const double d_yaw);
};
