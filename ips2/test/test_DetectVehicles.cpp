#include "catch.hpp"
#include "DetectVehicles.hpp"

TEST_CASE("apply_oneVehicle_detectedCorrectly")
{
    VehiclePointSet vehicle_point_set;
    vehicle_point_set.back_left  = cv::Point2d(-0.082, 0.017);
    vehicle_point_set.back_right = cv::Point2d(-0.082, -0.017);
    vehicle_point_set.center     = cv::Point2d(0, 0);
    vehicle_point_set.front      = cv::Point2d(0.082, 0);
    DetectVehicles detect_vehicles(vehicle_point_set);

    FloorPoints floor_points;
    floor_points.timestamp = 1000;
    floor_points.points = {vehicle_point_set.back_left,
                           vehicle_point_set.back_right,
                           vehicle_point_set.center,
                           vehicle_point_set.front};

    VehiclePoints vehicle_points_act = detect_vehicles.apply(floor_points);
    VehiclePoints vehicle_points_exp;
    vehicle_points_exp.timestamp = floor_points.timestamp;
    vehicle_points_exp.vehicles = {vehicle_point_set};
    CHECK(vehicle_points_act.timestamp == vehicle_points_exp.timestamp);
    CHECK(vehicle_points_act.vehicles.front().back_left == vehicle_points_exp.vehicles.front().back_left);
    CHECK(vehicle_points_act.vehicles.front().back_right == vehicle_points_exp.vehicles.front().back_right);
    CHECK(vehicle_points_act.vehicles.front().center == vehicle_points_exp.vehicles.front().center);
    CHECK(vehicle_points_act.vehicles.front().front == vehicle_points_exp.vehicles.front().front);
}