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
#include "DetectVehicles.hpp"
#include "cpm/Logging.hpp"

TEST_CASE("TEST_apply_WITH_0_points_SHOULD_not_crash")
{
    // Setup

    cpm::Logging::Instance().set_id("ips_pipeline");

    // Input
    FloorPoints floor_points;
    floor_points.timestamp = 1000;
    floor_points.points = {};
    // Expected result
    VehiclePoints vehicle_points_exp;
    vehicle_points_exp.timestamp = floor_points.timestamp;
    vehicle_points_exp.vehicles = {};
    // Test
    DetectVehicles detect_vehicles(0.164530613, 0.034);
    VehiclePoints vehicle_points_act = detect_vehicles.apply(floor_points);

    CHECK(vehicle_points_act.timestamp == vehicle_points_exp.timestamp);
    CHECK(vehicle_points_act.vehicles.size()==vehicle_points_exp.vehicles.size());
}

TEST_CASE("TEST_apply_WITH_3_points_of_1_vehicle_SHOULD_detect_1_vehicle")
{
    // Setup    
    // Vehicle points at same position as base geometry
    VehiclePointSet vehicle_point_set;
    vehicle_point_set.back_left  = cv::Point2d(-0.082, 0.017);
    vehicle_point_set.back_right = cv::Point2d(-0.082, -0.017);
    vehicle_point_set.front      = cv::Point2d(0.082, 0);

    // Input
    // Create input floor points
    FloorPoints floor_points;
    floor_points.timestamp = 1000;
    floor_points.points = {vehicle_point_set.back_left,
                           vehicle_point_set.back_right,
                           vehicle_point_set.front};

    // Expected Result
    VehiclePoints vehicle_points_exp;
    vehicle_points_exp.timestamp = floor_points.timestamp;
    vehicle_points_exp.vehicles = {vehicle_point_set};

    // Test
    cv::Point2d vec_front_to_rear = vehicle_point_set.back_left - vehicle_point_set.front;
    double d_front_back = std::sqrt(vec_front_to_rear.dot(vec_front_to_rear));
    cv::Point2d vec_rear_to_rear = vehicle_point_set.back_left - vehicle_point_set.back_right;
    double d_back_back = std::sqrt(vec_rear_to_rear.dot(vec_rear_to_rear));
    DetectVehicles detect_vehicles(d_front_back, d_back_back);
    VehiclePoints vehicle_points_act = detect_vehicles.apply(floor_points);

    CHECK(vehicle_points_act.timestamp == vehicle_points_exp.timestamp);
    REQUIRE(vehicle_points_act.vehicles.size()==1);
    CHECK(vehicle_points_act.vehicles[0].back_left == vehicle_points_exp.vehicles[0].back_left);
    CHECK(vehicle_points_act.vehicles[0].back_right == vehicle_points_exp.vehicles[0].back_right);
    REQUIRE(vehicle_points_act.vehicles[0].center_present == 0);
    CHECK(vehicle_points_act.vehicles[0].front == vehicle_points_exp.vehicles[0].front);
}


TEST_CASE("TEST_apply_WITH_4_points_of_1_vehicle_translated_SHOULD_detect_1_vehicle")
{
    // Setup
    VehiclePointSet vehicle_point_set;
    vehicle_point_set.back_left  = cv::Point2d(-0.07265, 0.017);
    vehicle_point_set.back_right = cv::Point2d(-0.07265, -0.017);
    vehicle_point_set.center     = cv::Point2d(0, 0);
    vehicle_point_set.front      = cv::Point2d(0.091, 0);
    
    // Translate vehicle points
    cv::Point2d translation(15.23, -983.7);
    vehicle_point_set.back_left  += translation;
    vehicle_point_set.back_right += translation;
    vehicle_point_set.center     += translation;
    vehicle_point_set.front      += translation;

    // Input
    // Create input floor points
    FloorPoints floor_points;
    floor_points.timestamp = 1000;
    floor_points.points = {vehicle_point_set.back_left,
                           vehicle_point_set.center,
                           vehicle_point_set.back_right,
                           vehicle_point_set.front};

    // Expected result
    VehiclePoints vehicle_points_exp;
    vehicle_points_exp.timestamp = floor_points.timestamp;
    vehicle_points_exp.vehicles = {vehicle_point_set};

    // Test
    cv::Point2d vec_front_to_rear = vehicle_point_set.back_left - vehicle_point_set.front;
    double d_front_back = std::sqrt(vec_front_to_rear.dot(vec_front_to_rear));
    cv::Point2d vec_rear_to_rear = vehicle_point_set.back_left - vehicle_point_set.back_right;
    double d_back_back = std::sqrt(vec_rear_to_rear.dot(vec_rear_to_rear));
    DetectVehicles detect_vehicles(d_front_back, d_back_back);
    VehiclePoints vehicle_points_act = detect_vehicles.apply(floor_points);
    
    CHECK(vehicle_points_act.timestamp == vehicle_points_exp.timestamp);
    REQUIRE(vehicle_points_act.vehicles.size()==1);
    CHECK(vehicle_points_act.vehicles[0].back_left == vehicle_points_exp.vehicles[0].back_left);
    CHECK(vehicle_points_act.vehicles[0].back_right == vehicle_points_exp.vehicles[0].back_right);
    REQUIRE(vehicle_points_act.vehicles[0].center_present);
    CHECK(vehicle_points_act.vehicles[0].center == vehicle_points_exp.vehicles[0].center);
    CHECK(vehicle_points_act.vehicles[0].front == vehicle_points_exp.vehicles[0].front);
}


TEST_CASE("TEST_apply_WITH_4_points_of_1_vehicle_translated_and_rotated_SHOULD_detect_1_vehicle")
{
    // Setup
    VehiclePointSet vehicle_point_set;
    vehicle_point_set.back_left  = cv::Point2d(-0.07265, 0.017);
    vehicle_point_set.back_right = cv::Point2d(-0.07265, -0.017);
    vehicle_point_set.center     = cv::Point2d(0, 0);
    vehicle_point_set.front      = cv::Point2d(0.091, 0);
    // Translate vehicle points
    cv::Point2d translation(0, 20.1);
    vehicle_point_set.back_left  += translation;
    vehicle_point_set.back_right += translation;
    vehicle_point_set.center     += translation;
    vehicle_point_set.front      += translation;
    // Rotate vehicle points
    double angle = -2.123;
    cv::Matx22d rotation(2, 2);
    rotation(0, 0) = std::cos(angle);
    rotation(1, 1) = rotation(0, 0);
    rotation(0, 1) = -std::sin(angle);
    rotation(1, 0) = -rotation(0, 1);
    vehicle_point_set.back_left  = rotation * vehicle_point_set.back_left;
    vehicle_point_set.back_right = rotation * vehicle_point_set.back_right;
    vehicle_point_set.center     = rotation * vehicle_point_set.center;
    vehicle_point_set.front      = rotation * vehicle_point_set.front;
    
    // Input
    // Create input floor points
    FloorPoints floor_points;
    floor_points.timestamp = 1000;
    floor_points.points = {vehicle_point_set.front,
                           vehicle_point_set.back_left,
                           vehicle_point_set.center,
                           vehicle_point_set.back_right};

    // Expected result
    VehiclePoints vehicle_points_exp;
    vehicle_points_exp.timestamp = floor_points.timestamp;
    vehicle_points_exp.vehicles = {vehicle_point_set};

    // Test
    cv::Point2d vec_front_to_rear = vehicle_point_set.back_left - vehicle_point_set.front;
    double d_front_back = std::sqrt(vec_front_to_rear.dot(vec_front_to_rear));
    cv::Point2d vec_rear_to_rear = vehicle_point_set.back_left - vehicle_point_set.back_right;
    double d_back_back = std::sqrt(vec_rear_to_rear.dot(vec_rear_to_rear));
    DetectVehicles detect_vehicles(d_front_back, d_back_back);
    VehiclePoints vehicle_points_act = detect_vehicles.apply(floor_points);
    
    CHECK(vehicle_points_act.timestamp == vehicle_points_exp.timestamp);
    REQUIRE(vehicle_points_act.vehicles.size()==1);
    CHECK(vehicle_points_act.vehicles[0].back_left == vehicle_points_exp.vehicles[0].back_left);
    CHECK(vehicle_points_act.vehicles[0].back_right == vehicle_points_exp.vehicles[0].back_right);
    REQUIRE(vehicle_points_act.vehicles[0].center_present);
    CHECK(vehicle_points_act.vehicles[0].center == vehicle_points_exp.vehicles[0].center);
    CHECK(vehicle_points_act.vehicles[0].front == vehicle_points_exp.vehicles[0].front);
}


TEST_CASE("TEST_apply_WITH_8_points_of_2_vehicles_SHOULD_detect_2_vehicles")
{
    // Setup
    VehiclePointSet vehicle_point_set_1;
    vehicle_point_set_1.back_left  = cv::Point2d(-0.07265, 0.017);
    vehicle_point_set_1.back_right = cv::Point2d(-0.07265, -0.017);
    vehicle_point_set_1.center     = cv::Point2d(0, 0);
    vehicle_point_set_1.front      = cv::Point2d(0.091, 0);
    VehiclePointSet vehicle_point_set_2 = vehicle_point_set_1;
    // Translate vehicle points
    cv::Point2d translation(0, 20.1);
    vehicle_point_set_2.back_left  += translation;
    vehicle_point_set_2.back_right += translation;
    vehicle_point_set_2.center     += translation;
    vehicle_point_set_2.front      += translation;
    // Rotate vehicle points
    double angle = -2.123;
    cv::Matx22d rotation(2, 2);
    rotation(0, 0) = std::cos(angle);
    rotation(1, 1) = rotation(0, 0);
    rotation(0, 1) = -std::sin(angle);
    rotation(1, 0) = -rotation(0, 1);
    vehicle_point_set_1.back_left  = rotation * vehicle_point_set_1.back_left;
    vehicle_point_set_1.back_right = rotation * vehicle_point_set_1.back_right;
    vehicle_point_set_1.center     = rotation * vehicle_point_set_1.center;
    vehicle_point_set_1.front      = rotation * vehicle_point_set_1.front;
    
    // Input
    // Create input floor points
    FloorPoints floor_points;
    floor_points.timestamp = 1041500;
    floor_points.points = {vehicle_point_set_2.back_left,
                           vehicle_point_set_2.center,
                           vehicle_point_set_1.back_left,
                           vehicle_point_set_1.back_right,
                           vehicle_point_set_2.front,
                           vehicle_point_set_1.center,
                           vehicle_point_set_2.back_right,
                           vehicle_point_set_1.front};

    // Expected result
    VehiclePoints vehicle_points_exp;
    vehicle_points_exp.timestamp = floor_points.timestamp;
    vehicle_points_exp.vehicles = {vehicle_point_set_1, vehicle_point_set_2};

    // Test
    cv::Point2d vec_front_to_rear = vehicle_point_set_1.back_left - vehicle_point_set_1.front;
    double d_front_back = std::sqrt(vec_front_to_rear.dot(vec_front_to_rear));
    cv::Point2d vec_rear_to_rear = vehicle_point_set_1.back_left - vehicle_point_set_1.back_right;
    double d_back_back = std::sqrt(vec_rear_to_rear.dot(vec_rear_to_rear));
    DetectVehicles detect_vehicles(d_front_back, d_back_back);
    VehiclePoints vehicle_points_act = detect_vehicles.apply(floor_points);
    
    CHECK(vehicle_points_act.timestamp == vehicle_points_exp.timestamp);
    REQUIRE(vehicle_points_act.vehicles.size()==2);
    CHECK(((vehicle_points_act.vehicles[0].back_left == vehicle_points_exp.vehicles[0].back_left && 
            vehicle_points_act.vehicles[1].back_left == vehicle_points_exp.vehicles[1].back_left) ||
           (vehicle_points_act.vehicles[0].back_left == vehicle_points_exp.vehicles[1].back_left && 
            vehicle_points_act.vehicles[1].back_left == vehicle_points_exp.vehicles[0].back_left)));
    
    CHECK(((vehicle_points_act.vehicles[0].back_right == vehicle_points_exp.vehicles[0].back_right && 
            vehicle_points_act.vehicles[1].back_right == vehicle_points_exp.vehicles[1].back_right) ||
           (vehicle_points_act.vehicles[0].back_right == vehicle_points_exp.vehicles[1].back_right && 
            vehicle_points_act.vehicles[1].back_right == vehicle_points_exp.vehicles[0].back_right)));
    
    CHECK(((vehicle_points_act.vehicles[0].front == vehicle_points_exp.vehicles[0].front && 
            vehicle_points_act.vehicles[1].front == vehicle_points_exp.vehicles[1].front) ||
           (vehicle_points_act.vehicles[0].front == vehicle_points_exp.vehicles[1].front && 
            vehicle_points_act.vehicles[1].front == vehicle_points_exp.vehicles[0].front)));
    
    REQUIRE(vehicle_points_act.vehicles[0].center_present);
    REQUIRE(vehicle_points_act.vehicles[1].center_present);

    CHECK(((vehicle_points_act.vehicles[0].center == vehicle_points_exp.vehicles[0].center && 
            vehicle_points_act.vehicles[1].center == vehicle_points_exp.vehicles[1].center) ||
           (vehicle_points_act.vehicles[0].center == vehicle_points_exp.vehicles[1].center && 
            vehicle_points_act.vehicles[1].center == vehicle_points_exp.vehicles[0].center)));
}



TEST_CASE("TEST_apply_WITH_4_points_1_vehicle_and_ghost_vehicle_SHOULD_detect_0_vehicle")
{
    /**          vehicle 1
     *           ---------
     *           o   ^    
     *   o           |-> o
     *           o        
     *   ---------
     *   ghost veh
     */
    // Setup
    VehiclePointSet vehicle_point_set_1;
    vehicle_point_set_1.back_left  = cv::Point2d(-0.07265, 0.017);
    vehicle_point_set_1.back_right = cv::Point2d(-0.07265, -0.017);
    vehicle_point_set_1.center     = cv::Point2d(0, 0);
    vehicle_point_set_1.front      = cv::Point2d(0.091, 0);
    
    // Translate vehicle points
    cv::Point2d ghost_point(-0.23621, 0);
    
    // Input
    // Create input floor points
    FloorPoints floor_points;
    floor_points.timestamp = 94;
    // transmit ghost vehicle first to make it hard for algorithm
    floor_points.points = {ghost_point,
                           vehicle_point_set_1.front,
                           vehicle_point_set_1.back_left,
                           vehicle_point_set_1.back_right};

    // Expected result
    VehiclePoints vehicle_points_exp;
    vehicle_points_exp.timestamp = floor_points.timestamp;
    vehicle_points_exp.vehicles = {};

    // Test
    cv::Point2d vec_front_to_rear = vehicle_point_set_1.back_left - vehicle_point_set_1.front;
    double d_front_back = std::sqrt(vec_front_to_rear.dot(vec_front_to_rear));
    cv::Point2d vec_rear_to_rear = vehicle_point_set_1.back_left - vehicle_point_set_1.back_right;
    double d_back_back = std::sqrt(vec_rear_to_rear.dot(vec_rear_to_rear));
    DetectVehicles detect_vehicles(d_front_back, d_back_back);
    VehiclePoints vehicle_points_act = detect_vehicles.apply(floor_points);
    
    CHECK(vehicle_points_act.timestamp == vehicle_points_exp.timestamp);
    CHECK(vehicle_points_act.vehicles.size()==0);
}

TEST_CASE("TEST_apply_WITH_7_points_of_2_vehicles_and_ghost_vehicle_SHOULD_detect_2_vehicles")
{
    /**     vehicle 1       vehicle 2
     *      ---------       ---------
     *      o   ^           o
     *          |-> o           o   o
     *      o               o
     *              ---------
     *              ghost veh
     */
    // Setup
    VehiclePointSet vehicle_point_set_1;
    vehicle_point_set_1.back_left  = cv::Point2d(-0.07265, 0.017);
    vehicle_point_set_1.back_right = cv::Point2d(-0.07265, -0.017);
    vehicle_point_set_1.center     = cv::Point2d(0, 0);
    vehicle_point_set_1.front      = cv::Point2d(0.091, 0);
    VehiclePointSet vehicle_point_set_2 = vehicle_point_set_1;
    
    // Translate vehicle points
    cv::Point2d translation(0.32712, 0);
    vehicle_point_set_2.back_left  += translation;
    vehicle_point_set_2.back_right += translation;
    vehicle_point_set_2.center     += translation;
    vehicle_point_set_2.front      += translation;
    
    // Input
    // Create input floor points
    FloorPoints floor_points;
    floor_points.timestamp = 94;
    // transmit ghost vehicle first to make it hard for algorithm
    floor_points.points = {vehicle_point_set_2.back_left,
                           vehicle_point_set_2.back_right,
                           vehicle_point_set_1.front,
                           vehicle_point_set_2.center,
                           vehicle_point_set_1.back_left,
                           vehicle_point_set_1.back_right,
                           vehicle_point_set_2.front};

    // Expected result
    VehiclePoints vehicle_points_exp;
    vehicle_points_exp.timestamp = floor_points.timestamp;
    vehicle_points_exp.vehicles = {vehicle_point_set_1, vehicle_point_set_2};

    // Test
    cv::Point2d vec_front_to_rear = vehicle_point_set_1.back_left - vehicle_point_set_1.front;
    double d_front_back = std::sqrt(vec_front_to_rear.dot(vec_front_to_rear));
    cv::Point2d vec_rear_to_rear = vehicle_point_set_1.back_left - vehicle_point_set_1.back_right;
    double d_back_back = std::sqrt(vec_rear_to_rear.dot(vec_rear_to_rear));
    DetectVehicles detect_vehicles(d_front_back, d_back_back);
    VehiclePoints vehicle_points_act = detect_vehicles.apply(floor_points);
    
    CHECK(vehicle_points_act.timestamp == vehicle_points_exp.timestamp);
    REQUIRE(vehicle_points_act.vehicles.size()==2);
    CHECK(((vehicle_points_act.vehicles[0].back_left == vehicle_points_exp.vehicles[0].back_left && 
            vehicle_points_act.vehicles[1].back_left == vehicle_points_exp.vehicles[1].back_left) ||
           (vehicle_points_act.vehicles[0].back_left == vehicle_points_exp.vehicles[1].back_left && 
            vehicle_points_act.vehicles[1].back_left == vehicle_points_exp.vehicles[0].back_left)));
    
    CHECK(((vehicle_points_act.vehicles[0].back_right == vehicle_points_exp.vehicles[0].back_right && 
            vehicle_points_act.vehicles[1].back_right == vehicle_points_exp.vehicles[1].back_right) ||
           (vehicle_points_act.vehicles[0].back_right == vehicle_points_exp.vehicles[1].back_right && 
            vehicle_points_act.vehicles[1].back_right == vehicle_points_exp.vehicles[0].back_right)));
    
    CHECK(((vehicle_points_act.vehicles[0].front == vehicle_points_exp.vehicles[0].front && 
            vehicle_points_act.vehicles[1].front == vehicle_points_exp.vehicles[1].front) ||
           (vehicle_points_act.vehicles[0].front == vehicle_points_exp.vehicles[1].front && 
            vehicle_points_act.vehicles[1].front == vehicle_points_exp.vehicles[0].front)));
    
    // one vehicle has center point
    REQUIRE(((vehicle_points_act.vehicles[0].center_present && !vehicle_points_act.vehicles[1].center_present) ||
             (vehicle_points_act.vehicles[1].center_present && !vehicle_points_act.vehicles[0].center_present)));
    CHECK(((vehicle_points_act.vehicles[0].center == vehicle_points_exp.vehicles[1].center || 
            vehicle_points_act.vehicles[1].center == vehicle_points_exp.vehicles[1].center)));
}

TEST_CASE("TEST_apply_WITH_8_points_of_2_vehicles_and_ghost_vehicle_SHOULD_detect_2_vehicles")
{
    /**     vehicle 1   vehicle 2
     *      ---------   ---------
     *      o           o
     *          o   o       o   o
     *      o           o
     *          ---------
     *          ghost veh
     */
    // Setup
    VehiclePointSet vehicle_point_set_1;
    vehicle_point_set_1.back_left  = cv::Point2d(-0.07265, 0.017);
    vehicle_point_set_1.back_right = cv::Point2d(-0.07265, -0.017);
    vehicle_point_set_1.center     = cv::Point2d(0, 0);
    vehicle_point_set_1.front      = cv::Point2d(0.091, 0);
    VehiclePointSet vehicle_point_set_2 = vehicle_point_set_1;
    
    // Translate vehicle points
    cv::Point2d translation(0.23, 0);
    vehicle_point_set_2.back_left  += translation;
    vehicle_point_set_2.back_right += translation;
    vehicle_point_set_2.center     += translation;
    vehicle_point_set_2.front      += translation;


    // Input
    // Create input floor points
    FloorPoints floor_points;
    floor_points.timestamp = 94;
    // transmit ghost vehicle first to make it hard for algorithm
    floor_points.points = {vehicle_point_set_2.back_left,
                           vehicle_point_set_2.back_right,
                           vehicle_point_set_1.front,
                           vehicle_point_set_2.center,
                           vehicle_point_set_1.back_left,
                           vehicle_point_set_1.back_right,
                           vehicle_point_set_2.front,
                           vehicle_point_set_1.center};

    // Expected result
    VehiclePoints vehicle_points_exp;
    vehicle_points_exp.timestamp = floor_points.timestamp;
    vehicle_points_exp.vehicles = {vehicle_point_set_1, vehicle_point_set_2};

    // Test
    cv::Point2d vec_front_to_rear = vehicle_point_set_1.back_left - vehicle_point_set_1.front;
    double d_front_back = std::sqrt(vec_front_to_rear.dot(vec_front_to_rear));
    cv::Point2d vec_rear_to_rear = vehicle_point_set_1.back_left - vehicle_point_set_1.back_right;
    double d_back_back = std::sqrt(vec_rear_to_rear.dot(vec_rear_to_rear));
    DetectVehicles detect_vehicles(d_front_back, d_back_back);
    VehiclePoints vehicle_points_act = detect_vehicles.apply(floor_points);
    
    CHECK(vehicle_points_act.timestamp == vehicle_points_exp.timestamp);
    REQUIRE(vehicle_points_act.vehicles.size()==2);
    CHECK(((vehicle_points_act.vehicles[0].back_left == vehicle_points_exp.vehicles[0].back_left && 
            vehicle_points_act.vehicles[1].back_left == vehicle_points_exp.vehicles[1].back_left) ||
           (vehicle_points_act.vehicles[0].back_left == vehicle_points_exp.vehicles[1].back_left && 
            vehicle_points_act.vehicles[1].back_left == vehicle_points_exp.vehicles[0].back_left)));
    
    CHECK(((vehicle_points_act.vehicles[0].back_right == vehicle_points_exp.vehicles[0].back_right && 
            vehicle_points_act.vehicles[1].back_right == vehicle_points_exp.vehicles[1].back_right) ||
           (vehicle_points_act.vehicles[0].back_right == vehicle_points_exp.vehicles[1].back_right && 
            vehicle_points_act.vehicles[1].back_right == vehicle_points_exp.vehicles[0].back_right)));
    
    CHECK(((vehicle_points_act.vehicles[0].front == vehicle_points_exp.vehicles[0].front && 
            vehicle_points_act.vehicles[1].front == vehicle_points_exp.vehicles[1].front) ||
           (vehicle_points_act.vehicles[0].front == vehicle_points_exp.vehicles[1].front && 
            vehicle_points_act.vehicles[1].front == vehicle_points_exp.vehicles[0].front)));
    
    // both vehicles have center point
    REQUIRE((vehicle_points_act.vehicles[0].center_present && vehicle_points_act.vehicles[1].center_present));
    CHECK(((vehicle_points_act.vehicles[0].center == vehicle_points_exp.vehicles[1].center || 
            vehicle_points_act.vehicles[1].center == vehicle_points_exp.vehicles[1].center)));
}