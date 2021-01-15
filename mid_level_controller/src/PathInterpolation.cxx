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

#include "PathInterpolation.hpp"
#include <cmath>

PathInterpolation::PathInterpolation(
    const double s_queried,
    const PathPoint start_point,
    const PathPoint end_point
) 
{

    const double s_start = double(start_point.s());
    const double s_end = double(end_point.s());

    const double delta_s = s_end - s_start;
    const double tau = (s_queried - s_start) / delta_s;
    
    const double tau2 = tau * tau;
    const double tau3 = tau * tau2;
    
    const double position_start_x = start_point.pose().x();
    const double position_start_y = start_point.pose().y();
    const double position_end_x = end_point.pose().x();
    const double position_end_y = end_point.pose().y();
    
    const double velocity_start_x = cos( start_point.pose().yaw() ) * delta_s;
    const double velocity_start_y = sin( start_point.pose().yaw() ) * delta_s;
    const double velocity_end_x = cos( end_point.pose().yaw() ) * delta_s;
    const double velocity_end_y = sin( end_point.pose().yaw() ) * delta_s;
    
    
    // Hermite spline coefficients
    const double p0 = 2*tau3 - 3*tau2 + 1;
    const double m0 = tau3 - 2*tau2 + tau;
    const double p1 = -2*tau3 + 3*tau2;
    const double m1 = tau3 - tau2;
    
    // Hermite spline derivative coefficients
    const double dp0 = 6*tau2 - 6*tau;
    const double dm0 = 3*tau2 - 4*tau + 1;
    const double dp1 = -6*tau2 + 6*tau;
    const double dm1 = 3*tau2 - 2*tau;
    
    // Hermite spline second derivative coefficients
    const double ddp0 = 12*tau - 6;
    const double ddm0 = 6*tau - 4;
    const double ddp1 = -12*tau + 6;
    const double ddm1 = 6*tau - 2;    
    
    position_x     =  position_start_x *   p0 + velocity_start_x *   m0 + position_end_x *   p1 + velocity_end_x *   m1;
    position_y     =  position_start_y *   p0 + velocity_start_y *   m0 + position_end_y *   p1 + velocity_end_y *   m1;
    velocity_x     = (position_start_x *  dp0 + velocity_start_x *  dm0 + position_end_x *  dp1 + velocity_end_x *  dm1) / delta_s;
    velocity_y     = (position_start_y *  dp0 + velocity_start_y *  dm0 + position_end_y *  dp1 + velocity_end_y *  dm1) / delta_s;
    acceleration_x = (position_start_x * ddp0 + velocity_start_x * ddm0 + position_end_x * ddp1 + velocity_end_x * ddm1) / (delta_s*delta_s);
    acceleration_y = (position_start_y * ddp0 + velocity_start_y * ddm0 + position_end_y * ddp1 + velocity_end_y * ddm1) / (delta_s*delta_s);
    
    yaw = atan2(velocity_y, velocity_x);
    speed = sqrt(velocity_x*velocity_x + velocity_y*velocity_y);
    curvature = (velocity_x * acceleration_y - velocity_y * acceleration_x) / (speed*speed*speed);
}