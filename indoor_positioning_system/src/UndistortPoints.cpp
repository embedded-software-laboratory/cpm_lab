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

#include "UndistortPoints.hpp"
#include <cassert>

/**
 * \file UndistortPoints.cpp
 * \ingroup ips
 */

/**
 * \brief TODO
 * \ingroup ips
 */
#define N_CALIBRATION_TERMS (15)

UndistortPoints::UndistortPoints(
    std::vector<double> _calibration_x, 
    std::vector<double> _calibration_y
)
:calibration_x(_calibration_x)
,calibration_y(_calibration_y)
{
    assert(calibration_x.size() == N_CALIBRATION_TERMS);
    assert(calibration_y.size() == N_CALIBRATION_TERMS);
}


FloorPoints UndistortPoints::apply(LedPoints led_points)
{
    FloorPoints result;
    result.timestamp = led_points.time_stamp().nanoseconds();

    for(auto image_point:led_points.led_points())
    {
        const double image_x = image_point.x() / 2048.0;
        const double image_y = image_point.y() / 2048.0;

        // Calculate monomials
        const double ix1 = image_x;
        const double ix2 = ix1 * image_x;
        const double ix3 = ix2 * image_x;
        const double ix4 = ix3 * image_x;

        const double iy1 = image_y;
        const double iy2 = iy1 * image_y;
        const double iy3 = iy2 * image_y;
        const double iy4 = iy3 * image_y;

        // Calibration based on two dimensional, 4th order polynomial
        const double features[] = {
            1, 
            ix1, iy1, 
            ix2, ix1 * iy1, iy2, 
            ix3, ix2 * iy1, ix1 * iy2, iy3,
            ix4, ix3 * iy1, ix2 * iy2, ix1 * iy3, iy4
        };

        double floor_x = 0;
        double floor_y = 0;
        for (int i = 0; i < N_CALIBRATION_TERMS; ++i)
        {
            floor_x += features[i] * calibration_x[i];
            floor_y += features[i] * calibration_y[i];
        }

        // The following calculation corrects for the error that is introduced, because
        // the LEDs are not on the ground, but a small distance above it.
        // This calibration does not need to be very precise, since another measurement based
        // calibration for the poses will correct small errors.
        const double camera_x = 2.27;
        const double camera_y = 1.94;
        const double camera_z = 3.17;
        const double LED_z = 0.058;
        const double scale_factor = (camera_z - LED_z) / camera_z;

        floor_x = scale_factor * (floor_x - camera_x) + camera_x;
        floor_y = scale_factor * (floor_y - camera_y) + camera_y;

        result.points.emplace_back(floor_x, floor_y);
    }

    return result;
}
    
