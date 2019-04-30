#include "DetectVehicles.hpp"
#include <tuple>
#include <iostream>

DetectVehicles::DetectVehicles(const VehiclePointSet &vehicle_point_geometry)
: vehicle_point_geometry(vehicle_point_geometry)
{

}

double length(cv::Point2d p)
{
    return sqrt(p.dot(p));
}

VehiclePoints DetectVehicles::apply(const FloorPoints &floor_points)
{
    VehiclePoints vehicle_points;
    vehicle_points.timestamp = floor_points.timestamp;
    
    // Dumb implementation for 1 vehicle
    if(floor_points.points.size() == 3 || floor_points.points.size() == 4)
    {
        std::vector< std::vector<size_t> > rear_edge_pair;
        std::vector< std::vector<size_t> > longitudinal_edge_pair;


        for (size_t i = 0; i < floor_points.points.size()-1; ++i)
        {
            for (size_t j = i+1; j < floor_points.points.size(); ++j)
            {
                const double dist = length(floor_points.points[i] - floor_points.points[j]);
                if (fabs(dist - 0.033) < 0.004)
                {
                    rear_edge_pair.emplace_back(std::vector<size_t>{i,j});
                }
                else if (fabs(dist - 0.164) < 0.012)
                {
                    longitudinal_edge_pair.emplace_back(std::vector<size_t>{i,j});
                }
            }
        }

        if(rear_edge_pair.size() == 1 && longitudinal_edge_pair.size() == 2)
        {
            //std::cout << "YAY!" << std::endl;

            int index_front = -1;
            for(auto i:std::vector<int>{0,1})
            {
                for(auto j:std::vector<int>{0,1})
                {
                    if(longitudinal_edge_pair[0][j] == longitudinal_edge_pair[1][i])
                    {
                        index_front = longitudinal_edge_pair[0][j];
                        assert(index_front != rear_edge_pair[0][0]);
                        assert(index_front != rear_edge_pair[0][1]);
                    }
                }
            }
            assert(index_front>=0);

            //std::cerr << "DBG " << __LINE__ << std::endl;
            //std::cerr << "index_front " << index_front << std::endl;
            //std::cerr << "rear_edge_pair[0][0] " << rear_edge_pair[0][0] << std::endl;
            //std::cerr << "rear_edge_pair[0][1] " << rear_edge_pair[0][1] << std::endl;

            const auto point_front = floor_points.points[index_front];
            const auto point_rear0 = floor_points.points[rear_edge_pair[0][0]];
            const auto point_rear1 = floor_points.points[rear_edge_pair[0][1]];

            cv::Point2d back_left;
            cv::Point2d back_right;
            //std::cerr << "DBG " << __LINE__ << std::endl;

            if((point_rear0 - point_front).cross(point_rear1 - point_front) > 0)
            {
                back_left = point_rear0;
                back_right = point_rear1;
            }
            else 
            {
                back_left = point_rear1;
                back_right = point_rear0;
            }
            //std::cerr << "DBG " << __LINE__ << std::endl;

            int index_center = -1;
            if(floor_points.points.size() == 4)
            {
                for (size_t i = 0; i < 4; ++i)
                {
                    if(i != index_front
                        && i != rear_edge_pair[0][0]
                        && i != rear_edge_pair[0][1])
                    {
                        index_center = i;
                        break;
                    }
                }
            }

            //std::cerr << "DBG " << __LINE__ << std::endl;


            VehiclePointSet vehicle_point_set;
            vehicle_point_set.back_left  = back_left;
            vehicle_point_set.back_right = back_right;

            if (index_center >= 0)
            {
                vehicle_point_set.center = floor_points.points[index_center];
            }
            //std::cerr << "DBG " << __LINE__ << std::endl;

            vehicle_point_set.front      = point_front;
            vehicle_point_set.center_present = floor_points.points.size() == 4;
            vehicle_points.vehicles.push_back(vehicle_point_set);


        }
        //std::cout << "=============================" << std::endl;
    }

    return vehicle_points;
}
