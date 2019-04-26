#pragma once
#include <vector>
#include <cstdint>
#include <opencv2/core.hpp>


struct FloorPoints
{
    uint64_t timestamp;
    std::vector<cv::Point2d> points; // LED points in floor coordinates (meters)
};


struct VehiclePointSet
{
    int id = -1;
    bool center_present = false;

    cv::Point2d front;
    cv::Point2d center;
    cv::Point2d back_left;
    cv::Point2d back_right;
};

struct VehiclePoints
{
    uint64_t timestamp;
    std::vector<VehiclePointSet> vehicles;
};

using VehiclePointTimeseries = std::vector<VehiclePoints>;