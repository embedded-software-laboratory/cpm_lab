#include "catch.hpp"
#include "DetectVehicles.hpp"

TEST_CASE("TEST_apply_WITH_4_points_of_1_vehicle_SHOULD_detect_1_vehicle")
{
    // Setup
    VehiclePointSet vehicle_point_geometry;
    vehicle_point_geometry.back_left  = cv::Point2d(-0.082, 0.017);
    vehicle_point_geometry.back_right = cv::Point2d(-0.082, -0.017);
    vehicle_point_geometry.center     = cv::Point2d(0, 0);
    vehicle_point_geometry.front      = cv::Point2d(0.082, 0);
    DetectVehicles detect_vehicles(vehicle_point_geometry);
    
    // Vehicle points at same position as base geometry
    VehiclePointSet vehicle_point_set = vehicle_point_geometry;

    // Input
    // Create input floor points
    FloorPoints floor_points;
    floor_points.timestamp = 1000;
    floor_points.points = {vehicle_point_set.back_left,
                           vehicle_point_set.back_right,
                           vehicle_point_set.center,
                           vehicle_point_set.front};

    // Expected Result
    VehiclePoints vehicle_points_exp;
    vehicle_points_exp.timestamp = floor_points.timestamp;
    vehicle_points_exp.vehicles = {vehicle_point_set};

    // Test
    VehiclePoints vehicle_points_act = detect_vehicles.apply(floor_points);
    CHECK(vehicle_points_act.timestamp == vehicle_points_exp.timestamp);
    CHECK(vehicle_points_act.vehicles.front().back_left == vehicle_points_exp.vehicles.front().back_left);
    CHECK(vehicle_points_act.vehicles.front().back_right == vehicle_points_exp.vehicles.front().back_right);
    CHECK(vehicle_points_act.vehicles.front().center == vehicle_points_exp.vehicles.front().center);
    CHECK(vehicle_points_act.vehicles.front().front == vehicle_points_exp.vehicles.front().front);
}


TEST_CASE("TEST_apply_WITH_4_points_of_1_vehicle_translated_SHOULD_detect_1_vehicle")
{
    // Setup
    VehiclePointSet vehicle_point_geometry;
    vehicle_point_geometry.back_left  = cv::Point2d(-0.082, 0.017);
    vehicle_point_geometry.back_right = cv::Point2d(-0.082, -0.017);
    vehicle_point_geometry.center     = cv::Point2d(0, 0);
    vehicle_point_geometry.front      = cv::Point2d(0.082, 0);
    DetectVehicles detect_vehicles(vehicle_point_geometry);

    // Translated vehicle point geometry
    VehiclePointSet vehicle_point_set = vehicle_point_geometry;
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
                           vehicle_point_set.back_right,
                           vehicle_point_set.center,
                           vehicle_point_set.front};

    // Expected result
    VehiclePoints vehicle_points_exp;
    vehicle_points_exp.timestamp = floor_points.timestamp;
    vehicle_points_exp.vehicles = {vehicle_point_set};

    // Test
    VehiclePoints vehicle_points_act = detect_vehicles.apply(floor_points);
    CHECK(vehicle_points_act.timestamp == vehicle_points_exp.timestamp);
    CHECK(vehicle_points_act.vehicles.front().back_left == vehicle_points_exp.vehicles.front().back_left);
    CHECK(vehicle_points_act.vehicles.front().back_right == vehicle_points_exp.vehicles.front().back_right);
    CHECK(vehicle_points_act.vehicles.front().center == vehicle_points_exp.vehicles.front().center);
    CHECK(vehicle_points_act.vehicles.front().front == vehicle_points_exp.vehicles.front().front);
}