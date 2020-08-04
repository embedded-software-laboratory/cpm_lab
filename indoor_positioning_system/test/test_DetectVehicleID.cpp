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

#include "catch.hpp"
#include "DetectVehicleID.hpp"



TEST_CASE("test_DetectVehicleID")
{
    // Create test data
    std::vector<uint8_t> vehicle_id_4_blink_sequence {0,0,0,0,0,0,0,1,1,0,0,0,0};
    std::vector<uint8_t> vehicle_id_9_blink_sequence {0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0};

    VehiclePointTimeseries input;
    for (int i = 0; i < 50; ++i) // Generate 50 samples == 1 second
    {
        // Generate two blinking, moving vehicles
        VehiclePointSet vehicle_id_4;
        vehicle_id_4.center_present = bool(vehicle_id_4_blink_sequence[i % vehicle_id_4_blink_sequence.size()]);
        vehicle_id_4.front      = cv::Point2d(0.08 * i,1.0);
        vehicle_id_4.center     = cv::Point2d(0.08 * i,1.0);
        vehicle_id_4.back_left  = cv::Point2d(0.08 * i,1.0);
        vehicle_id_4.back_right = cv::Point2d(0.08 * i,1.0);
        
        VehiclePointSet vehicle_id_9;
        vehicle_id_9.center_present = bool(vehicle_id_9_blink_sequence[i % vehicle_id_9_blink_sequence.size()]);
        vehicle_id_9.front      = cv::Point2d(2.0 - 0.07 * i,-1.0);
        vehicle_id_9.center     = cv::Point2d(2.0 - 0.07 * i,-1.0);
        vehicle_id_9.back_left  = cv::Point2d(2.0 - 0.07 * i,-1.0);
        vehicle_id_9.back_right = cv::Point2d(2.0 - 0.07 * i,-1.0);

        VehiclePoints vehicles;
        vehicles.timestamp = 123456 + i * 20000000;

        if(i % 2 == 0) // Flip the order, it should not matter
        {
            vehicles.vehicles.push_back(vehicle_id_4);
            vehicles.vehicles.push_back(vehicle_id_9);
        }
        else
        {
            vehicles.vehicles.push_back(vehicle_id_9);
            vehicles.vehicles.push_back(vehicle_id_4);
        }
        input.push_back(vehicles);
    }


    // Act
    DetectVehicleID detectVehicleID(
        std::vector<uint8_t>{ 1, 4, 7, 10, 13, 16, 7, 10, 13, 16, 19, 10, 13, 16, 19, 22, 13, 16, 19, 22, 25, 16, 19, 22, 25, 28 },
        std::vector<uint8_t>{ 0, 2, 2,  2,  2,  2, 5,  5,  5,  5,  5,  8,  8,  8,  8,  8, 11, 11, 11, 11, 11, 14, 14, 14, 14, 14 }
    );

    VehiclePoints result = detectVehicleID.apply(input);

    // Assert
    REQUIRE(result.vehicles.size() == 2);
    CHECK(((result.vehicles[0].id == 9 && result.vehicles[1].id == 4) 
        || (result.vehicles[0].id == 4 && result.vehicles[1].id == 9)));

}