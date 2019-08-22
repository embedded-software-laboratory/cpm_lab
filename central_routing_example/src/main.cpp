
#include "lane_graph.hpp"
#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "Pose2D.hpp"
#include <iostream>

class LaneGraphTools : public LaneGraph
{
public:
    bool map_match_pose(Pose2D pose, int &out_edge_index, int &out_edge_path_index);
};

bool LaneGraphTools::map_match_pose(Pose2D pose, int &out_edge_index, int &out_edge_path_index)
{
    bool match_found = false;
    double min_squared_distance = 1e300;

    for (int i_edge = 0; i_edge < n_edges; ++i_edge)
    {
        for (int i_path = 0; i_path < n_edge_path_nodes; ++i_path)
        {
            const double x = edges_x[i_edge][i_path];
            const double y = edges_y[i_edge][i_path];
            const double c = edges_cos[i_edge][i_path];
            const double s = edges_sin[i_edge][i_path];

            const double cos_delta_yaw = c * cos(pose.yaw()) + s * sin(pose.yaw());
            const double dx = x - pose.x();
            const double dy = y - pose.y();
            const double squared_distance = dx*dx + dy*dy;

            if( cos_delta_yaw > 0.7
                && squared_distance < 0.09
                && squared_distance < min_squared_distance)
            {
                match_found = true;
                out_edge_index = i_edge;
                out_edge_path_index = i_path;
                min_squared_distance = squared_distance;
            }
        }
    }
    return match_found;
}

int main(int argc, char *argv[])
{
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id("central_routing_example");
    const bool enable_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);

    LaneGraphTools laneGraph;
    std::cout << laneGraph.n_nodes << std::endl;

    Pose2D pose;
    pose.x(0.28);
    pose.y(2.05);
    pose.yaw(1.52);
    int out_edge_index = -1;
    int out_edge_path_index = -1;
    std::cout << laneGraph.map_match_pose(pose, out_edge_index, out_edge_path_index) << std::endl;
    
    std::cout << out_edge_index << std::endl;
    std::cout << out_edge_path_index << std::endl;
}