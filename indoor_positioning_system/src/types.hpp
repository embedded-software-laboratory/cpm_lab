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
#include <list>
#include <cstdint>
#include <opencv2/core.hpp>

/**
 * \struct FloorPoints
 * \brief TODO
 * \ingroup ips
 */
struct FloorPoints
{
    uint64_t timestamp;
    std::vector<cv::Point2d> points; // LED points in floor coordinates (meters)
};

/**
 * \struct VehiclePointSet
 * \brief TODO
 * \ingroup ips
 */
struct VehiclePointSet
{
    int id = 0;
    bool center_present = false;

    cv::Point2d front;
    cv::Point2d center;
    cv::Point2d back_left;
    cv::Point2d back_right;
};

/**
 * \struct VehiclePoints
 * \brief TODO
 * \ingroup ips
 */
struct VehiclePoints
{
    uint64_t timestamp;
    std::vector<VehiclePointSet> vehicles;
};

using VehiclePointTimeseries = std::list<VehiclePoints>;