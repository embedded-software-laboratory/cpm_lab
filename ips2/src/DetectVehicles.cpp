#include "DetectVehicles.hpp"

static inline double length(cv::Point2d p)
{
    return sqrt(p.dot(p));
}


DetectVehicles::DetectVehicles(const double &d_front_rear, const double &d_rear_rear)
: tolerance_front_rear(d_front_rear * point_distance_tolerance),
  tolerance_rear_rear(d_rear_rear * point_distance_tolerance),
  d_front_rear(d_front_rear),
  d_rear_rear(d_rear_rear)
{
}

VehiclePoints DetectVehicles::apply(const FloorPoints &floor_points) const
{

    const cv::Mat_<double> point_distances = calc_point_distances(floor_points.points);

    std::vector< std::array<std::size_t, 3> > vehicle_candidates = find_vehicle_candidates(floor_points.points,
                                                                                           point_distances);

    // // Matrix stores conflicts between vehicle candidates, e.g. when a point is used by both
    // cv::Mat_<int> mat_conflicts = determine_conflicts(vehicle_candidates);

    // TODO: Resolve conflicts
    // TODO: remaining_points -> list
    // copy floor_points, remove every vehicle point
    std::list<cv::Point2d> remaining_points = find_remaining_points(floor_points.points,
                                                                    vehicle_candidates);

    VehiclePoints vehicle_points;
    vehicle_points.timestamp = floor_points.timestamp;
    for (const std::array<std::size_t, 3> &vehicle_candidate : vehicle_candidates)
    {
        VehiclePointSet vehicle_point_set = assign_vehicle_points(floor_points.points,
                                                                  vehicle_candidate);
        find_center_point(vehicle_point_set, remaining_points);
        vehicle_points.vehicles.push_back(vehicle_point_set);
    }
    return vehicle_points;
}

cv::Mat_<double> DetectVehicles::calc_point_distances(const std::vector<cv::Point2d> &points) const
{
    cv::Mat_<double> point_distances = cv::Mat_<double>::zeros(points.size(), points.size());
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        for (std::size_t j = i+1; j < points.size(); ++j)
        {
            point_distances(i, j) = length(points[i] - points[j]);
            point_distances(j, i) = point_distances(i, j);
        }            
    }
    return point_distances;    
}


std::vector< std::array<std::size_t, 3> > DetectVehicles::find_vehicle_candidates(const std::vector<cv::Point2d> &points,
                                                                                  const cv::Mat_<double> &point_distances) const
{
    if(points.size() < 3) return {};

    std::vector< std::array<std::size_t, 3> > vehicle_candidates;
    for (std::size_t i = 0; i < points.size()-2; ++i)
    {
        for (std::size_t j = i+1; j < points.size()-1; ++j)
        {
            for (std::size_t k = j+1; k < points.size(); ++k)
            {
                
                int num_rear_rear = 0;
                int num_front_rear = 0;
                std::array<double, 3> distances = {point_distances(i, j),
                                                   point_distances(i, k),
                                                   point_distances(j, k)};
                // check distances
                for (const double &d : distances)
                {
                    // points in front-back distance?
                    if (std::fabs(d - d_front_rear) < tolerance_front_rear)
                    {
                        ++num_front_rear;
                    }
                    // points in back-back distance?
                    else if (std::fabs(d - d_rear_rear) < tolerance_rear_rear)
                    {
                        ++num_rear_rear;
                    }
                       
                }
                if (num_rear_rear == 1 && num_front_rear == 2)
                {
                    std::array<std::size_t, 3> vehicle_candidate = {i, j, k};
                    vehicle_candidates.push_back(vehicle_candidate);
                }
            }
        }   
    }
    return vehicle_candidates;
}

cv::Mat_<int> DetectVehicles::determine_conflicts(const std::vector< std::array<std::size_t, 3> > &vehicle_candidates) const
{
    cv::Mat_<int> mat_conflicts = cv::Mat_<int>::zeros(vehicle_candidates.size(), vehicle_candidates.size());
    for (std::size_t i = 0; i < vehicle_candidates.size()-1; ++i)
    {
        for (std::size_t j = i+1; j < vehicle_candidates.size(); ++j)
        {
            if (arrays_with_common_element(vehicle_candidates[i], vehicle_candidates[j]))
            {
                mat_conflicts(i, j) = 1;
                mat_conflicts(j, i) = 1;
            }
        }
    }
    return mat_conflicts;
}

bool DetectVehicles::arrays_with_common_element(const std::array<std::size_t, 3> &array_1,
                                                 const std::array<std::size_t, 3> &array_2) const
{
    for (const std::size_t &i : array_1)
    {
        for (const std::size_t &j : array_2)
        {
            if (i == j)
            {
                return true;
            }
        }
    }
    return false;
}


std::list<cv::Point2d> DetectVehicles::find_remaining_points(const std::vector<cv::Point2d> &floor_points,
                                                             const std::vector< std::array<std::size_t, 3> > &vehicle_candidates) const
{
    // determine indices belonging to vehicles
    std::vector<std::size_t> all_vehicle_indices;
    for (const auto &vehicle_candidate : vehicle_candidates)
    {
        for (const auto &idx : vehicle_candidate)
        {
            all_vehicle_indices.push_back(idx);
        }
    }
    std::sort(all_vehicle_indices.begin(), all_vehicle_indices.end());
    std::size_t vehicle_idx = 0;
    std::list<cv::Point2d> remaining_points;
    for (std::size_t i = 0; i < floor_points.size(); ++i)
    {
        // current index is assigned to a vehicle
        if (vehicle_idx < all_vehicle_indices.size() && i == all_vehicle_indices[vehicle_idx])
        {
            ++vehicle_idx;
        }
        // current index is NOT assigned to a vehicle
        else
        {
            remaining_points.push_back(floor_points[i]);
        }        
    }
    return remaining_points;
}

VehiclePointSet DetectVehicles::assign_vehicle_points(const std::vector<cv::Point2d> &floor_points,
                                                      const std::array<std::size_t, 3> &vehicle_candidate) const
{
    // find rear points
    std::vector<std::size_t> rear_edge_pair;
    for (std::size_t i = 0; i < vehicle_candidate.size() - 1; ++i)
    {
        for (std::size_t j = i + 1; j < vehicle_candidate.size(); ++j)
        {
            cv::Point2d point1 = floor_points[vehicle_candidate[i]];
            cv::Point2d point2 = floor_points[vehicle_candidate[j]];

            if (fabs(length(point1-point2) - d_rear_rear) < tolerance_rear_rear)
            {
                rear_edge_pair.push_back(vehicle_candidate[i]);
                rear_edge_pair.push_back(vehicle_candidate[j]);
            }
        }
    }
    assert(rear_edge_pair.size() == 2);

    // determine front point
    cv::Point2d front;
    for (const std::size_t i : vehicle_candidate)
    {
        if (i != rear_edge_pair[0] && i != rear_edge_pair[1])
        {
            front = floor_points[i];
        }
    }

    // determine rear left and right
    const auto rear0 = floor_points[rear_edge_pair[0]];
    const auto rear1 = floor_points[rear_edge_pair[1]];
    cv::Point2d rear_left;
    cv::Point2d rear_right;

    if((rear0 - front).cross(rear1 - front) > 0)
    {
        rear_left = rear0;
        rear_right = rear1;
    }
    else 
    {
        rear_left = rear1;
        rear_right = rear0;
    }

    VehiclePointSet vehicle_point_set;
    vehicle_point_set.front = front;
    vehicle_point_set.back_left = rear_left;
    vehicle_point_set.back_right = rear_right;    

    return vehicle_point_set;
}


void DetectVehicles::find_center_point(VehiclePointSet &vehicle_point_set,
                                       std::list<cv::Point2d> &remaining_points) const
{
    // weighted average: 70/152.5 for front point, 82.5/152.5/2 for rear points
    cv::Point2d point_center = 0.270491803 * (vehicle_point_set.back_left + vehicle_point_set.back_right)
                               + 0.459016393 * vehicle_point_set.front;
    for (auto it = remaining_points.begin(); it != remaining_points.end(); ++it)
    {
        if (fabs(length(*it - point_center)) < 0.03)
        {
            vehicle_point_set.center_present = 1;
            vehicle_point_set.center = *it;
            remaining_points.erase(it);
            return;
        }
    }
    vehicle_point_set.center_present = 0;
    return;
}