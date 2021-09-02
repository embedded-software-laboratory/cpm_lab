#include "PoseCalculation.hpp"
#include <cassert>

/**
 * \file PoseCalculation.cpp
 * \ingroup ips
 */

/**
 * \brief TODO
 * \ingroup ips
 */
#define N_POSE_CALIBRATION_TERMS (5)


PoseCalculation::PoseCalculation()
:calibration_px({1.0033,     0.0,   0.0792,      0.0,     0.0})
,calibration_py({   0.0,  1.0033,      0.0,   0.0792,     0.0})
,calibration_dx({   0.0,     0.0,      1.0,   0.0038,     0.0})
,calibration_dy({   0.0,     0.0,  -0.0054,      1.0,     0.0})
{
    assert(calibration_px.size() == N_POSE_CALIBRATION_TERMS);
    assert(calibration_py.size() == N_POSE_CALIBRATION_TERMS);
    assert(calibration_dx.size() == N_POSE_CALIBRATION_TERMS);
    assert(calibration_dy.size() == N_POSE_CALIBRATION_TERMS);
}

/**
 * \brief TODO
 * \param p TODO
 * \ingroup ips
 */
static inline double length(cv::Point2d p)
{
    return sqrt(p.dot(p));
}

std::vector<VehicleObservation> PoseCalculation::apply(const VehiclePoints &vehiclePoints)
{
    std::vector<VehicleObservation> result;

    for(const auto &vehicle : vehiclePoints.vehicles)
    {
        const cv::Point2d back = 0.5 * (vehicle.back_left + vehicle.back_right);
        const cv::Point2d front = vehicle.front;
        const cv::Point2d direction = front - back;
        const cv::Point2d direction_normalized = direction * (1.0 / length(direction));

        const double features[] = {
            back.x, back.y, direction_normalized.x, direction_normalized.y, 1
        };


        // Uncomment this when creating a new pose calibration
        /*
        std::cout << "pose_features ";
        for (int i = 0; i < N_POSE_CALIBRATION_TERMS; ++i) std::cout << features[i] << ",  ";
        std::cout << std::endl;
        */

    

        double ref_position_x = 0;
        double ref_position_y = 0;
        double ref_direction_x = 0;
        double ref_direction_y = 0;
        for (int i = 0; i < N_POSE_CALIBRATION_TERMS; ++i)
        {
            ref_position_x += features[i] * calibration_px[i];
            ref_position_y += features[i] * calibration_py[i];
            ref_direction_x += features[i] * calibration_dx[i];
            ref_direction_y += features[i] * calibration_dy[i];

        }

        VehicleObservation vehicleObservation;
        vehicleObservation.header().create_stamp().nanoseconds(vehiclePoints.timestamp);
        vehicleObservation.header().valid_after_stamp().nanoseconds(vehiclePoints.timestamp);
        vehicleObservation.pose().x(ref_position_x);
        vehicleObservation.pose().y(ref_position_y);
        vehicleObservation.pose().yaw(atan2(ref_direction_y, ref_direction_x));
        vehicleObservation.vehicle_id(vehicle.id);
        result.push_back(vehicleObservation);
    }


    return result;
}