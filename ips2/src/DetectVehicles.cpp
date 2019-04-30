#include "DetectVehicles.hpp"

DetectVehicles::DetectVehicles(const double &d_front_back, const double &d_back_back)
: d_front_back_min(d_front_back * (1 - point_distance_tolerance)),
  d_front_back_max(d_front_back * (1 + point_distance_tolerance)),
  d_back_back_min(d_back_back * (1 - point_distance_tolerance)),
  d_back_back_max(d_back_back * (1 + point_distance_tolerance))
{
}

VehiclePoints DetectVehicles::apply(const FloorPoints &floor_points) const
{
    VehiclePoints vehicle_points;
    vehicle_points.timestamp = floor_points.timestamp;

    // const cv::Mat_<double> point_distances = calc_point_distances(floor_points.points);
    // std::vector< std::array<int, 3> > vehicle_candidates = find_vehicle_candidates(floor_points.points,
    //                                                                                point_distances);
    // // Matrix stores conflicts between vehicle candidates, e.g. when a point is used by both
    // cv::Mat_<int> mat_conflicts = determine_conflicts(vehicle_candidates);

    // TODO: Resolve conflicts

    VehiclePointSet vehicle_point_set;
    vehicle_point_set.back_left  = cv::Point2d(0, 0);
    vehicle_point_set.back_right = cv::Point2d(0, 0);
    vehicle_point_set.center     = cv::Point2d(0, 0);
    vehicle_point_set.front      = cv::Point2d(0, 0);
    vehicle_points.vehicles.push_back(vehicle_point_set);
    return vehicle_points;
}

cv::Mat_<double> DetectVehicles::calc_point_distances(const std::vector<cv::Point2d> &points) const
{
    cv::Mat_<double> point_distances = cv::Mat_<double>::zeros(points.size(), points.size());
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        for (std::size_t j = i+1; j < points.size(); ++j)
        {
            point_distances(i, j) = points[i].dot(points[j]);
            // TODO: Symmetry
        }            
    }
    return point_distances;    
}


std::vector< std::array<int, 3> > DetectVehicles::find_vehicle_candidates(const std::vector<cv::Point2d> &points,
                                                                        const cv::Mat_<double> &point_distances) const
{
    std::vector< std::array<int, 3> > vehicle_candidates;
    for (std::size_t i = 0; i < points.size()-2; ++i)
    {
        for (std::size_t j = i+1; j < points.size()-1; ++j)
        {
            for (std::size_t k = j+1; k < points.size(); ++k)
            {
                
                int num_back_back = 0;
                int num_front_back = 0;
                std::array<double, 3> distances = {point_distances(i, j),
                                                   point_distances(i, k),
                                                   point_distances(j, k)};
                // check distances
                for (const double &d : distances)
                {
                    // check if points are close enough
                    if (d < d_front_back_max)
                    {
                        // distance in front-back tolerance interval?
                        if (d > d_front_back_min)
                        {
                            ++num_front_back;
                        }
                        // distance in back-back tolerance interval?
                        else if (d < d_back_back_max && d > d_back_back_min)
                        {
                            ++num_back_back;
                        }
                    }   
                }
                if (num_back_back == 1 && num_front_back == 2)
                {
                    std::array<int, 3> vehicle_candidate = {i, j, k};
                    vehicle_candidates.push_back(vehicle_candidate);
                }
            }
        }   
    }
    return vehicle_candidates;
}

cv::Mat_<int> DetectVehicles::determine_conflicts(const std::vector< std::array<int, 3> > &vehicle_candidates) const
{
    cv::Mat_<int> mat_conflicts = cv::Mat_<int>::zeros(vehicle_candidates.size(), vehicle_candidates.size());
    for (std::size_t i = 0; i < vehicle_candidates.size()-1; ++i)
    {
        for (std::size_t j = i+1; j < vehicle_candidates.size(); ++j)
        {
            if (arrays_with_similar_element(vehicle_candidates[i], vehicle_candidates[j]))
            {
                mat_conflicts(i, j) = 1;
                mat_conflicts(j, i) = 1;
            }
        }
    }
    return mat_conflicts;
}

bool DetectVehicles::arrays_with_similar_element(const std::array<int, 3> &array_1,
                                                 const std::array<int, 3> &array_2) const
{
    for (const int &i : array_1)
    {
        for (const int &j : array_2)
        {
            if (i == j)
            {
                return true;
            }
        }
    }
    return false;
}                                                             