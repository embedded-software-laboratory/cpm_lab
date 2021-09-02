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
    //! TODO
    uint64_t timestamp;

    //! TODO
    std::vector<cv::Point2d> points; // LED points in floor coordinates (meters)
};

/**
 * \struct VehiclePointSet
 * \brief TODO
 * \ingroup ips
 */
struct VehiclePointSet
{
    //! TODO
    int id = 0;
    //! TODO
    bool center_present = false;

    //! TODO
    cv::Point2d front;
    //! TODO
    cv::Point2d center;
    //! TODO
    cv::Point2d back_left;
    //! TODO
    cv::Point2d back_right;
};

/**
 * \struct VehiclePoints
 * \brief TODO
 * \ingroup ips
 */
struct VehiclePoints
{
    //! TODO
    uint64_t timestamp;
    //! TODO
    std::vector<VehiclePointSet> vehicles;
};

using VehiclePointTimeseries = std::list<VehiclePoints>;