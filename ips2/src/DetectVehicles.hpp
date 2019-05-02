#pragma once
#include <algorithm> // std::sort
#include <iostream> // debug
#include <list>
#include <vector>
#include "types.hpp"
#include <opencv2/core.hpp>

class DetectVehicles
{
public:
    DetectVehicles(const double &d_front_rear, const double &d_rear_rear);

    VehiclePoints apply(const FloorPoints &floor_points) const;

private:
    /** TODO: Move to appropriate file
     * @brief Point set defining LED geometry [m]
     * 
     * Point set defining the positions of the LEDs on a vehicle in [m]
     * in a coordinate frame with the origin in the center LED.
     * 
     * 
     *  back_left     
     *      o       y ^ 
     *                |
     *                o ---->      o  front
     *              center   x
     *      o
     *  back_right
     * 
     * From CAD 20190203 LEDholder:
     * back_left  = cv::Point2d(-0.07265, 0.017);
     * back_right = cv::Point2d(-0.07265, -0.017);
     * center     = cv::Point2d(0, 0);
     * front      = cv::Point2d(0.091, 0);
     * 
     */

    /**
     * @brief 
     * 
     * distance front to back
     * actual:   0.1628 - 0.1649
     * expected: 0.1645
     * deviation of factor 0.01
     * 
     * distance back left to back right
     * actual:   
     * expected: 0.034
     */
    const double point_distance_tolerance = 0.1;

    const double tolerance_front_rear;
    const double tolerance_rear_rear;
    const double d_front_rear;
    const double d_rear_rear;

    /**
     * @brief Calculate distances between points
     * 
     * @param points 
     * @return cv::Mat_<double> 
     */
    cv::Mat_<double> calc_point_distances(const std::vector<cv::Point2d> &points) const;


    std::vector< std::array<std::size_t, 3> > find_vehicle_candidates(const std::vector<cv::Point2d> &points,
                                                            const cv::Mat_<double> &point_distances) const;

    cv::Mat_<int> determine_conflicts(const std::vector< std::array<std::size_t, 3> > &vehicle_candidates) const;

    /**
     * @brief Check if two arrays have an element in common
     * 
     * @param array_1 
     * @param array_2 
     * @return true 
     * @return false 
     */
    bool arrays_with_common_element(const std::array<std::size_t, 3> &array_1, const std::array<std::size_t, 3> &array_2) const;

    std::list<cv::Point2d> find_remaining_points(const std::vector<cv::Point2d> &floor_points,
                                                 const std::vector< std::array<std::size_t, 3> > &vehicle_candidates) const;

    VehiclePointSet assign_vehicle_points(const std::vector<cv::Point2d> &floor_points,
                                          const std::array<std::size_t, 3> &vehicle_candidate) const;

    /**
     * @brief Find center point and remove from remaining point list
     * 
     * @param vehicle_point_set 
     * @param remaining_points 
     */
    void find_center_point(VehiclePointSet &vehicle_point_set,
                           std::list<cv::Point2d> &remaining_points) const;  
};