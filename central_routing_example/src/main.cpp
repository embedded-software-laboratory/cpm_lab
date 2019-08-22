
#include "lane_graph.hpp"
#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/AsyncReader.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "VehicleObservation.hpp"
#include <iostream>

using std::vector;

class LaneGraphTools : public LaneGraph
{
public:
    bool map_match_pose(Pose2D pose, int &out_edge_index, int &out_edge_path_index);
    vector<size_t> find_subsequent_edges(int edge_index);
};

bool LaneGraphTools::map_match_pose(Pose2D pose, int &out_edge_index, int &out_edge_path_index)
{
    bool match_found = false;
    double min_squared_distance = 1e300;

    for (size_t i_edge = 0; i_edge < n_edges; ++i_edge)
    {
        for (size_t i_path = 0; i_path < n_edge_path_nodes; ++i_path)
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

vector<size_t> LaneGraphTools::find_subsequent_edges(int edge_index)
{
    vector<size_t> result;

    size_t subsequent_node_index = edges_end_index.at(edge_index);
    for (size_t i_edge = 0; i_edge < n_edges; ++i_edge)
    {
        if(edges_start_index[i_edge] == subsequent_node_index)
        {
            result.push_back(i_edge);
        }
    }

    return result;
}

int main(int argc, char *argv[])
{
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id("central_routing_example");
    const bool enable_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);

    uint8_t vehicle_id = 0;
    int current_edge_index = -1;
    int current_edge_path_index = -1;
    double current_speed = 0;

    LaneGraphTools laneGraph;

    // receive vehicle pose from IPS
    cpm::AsyncReader<VehicleObservation> vehicleObservations_reader(
        [&](dds::sub::LoanedSamples<VehicleObservation>& samples) {

            if (vehicle_id > 0) return;

            for(auto sample : samples) 
            {
                if(sample.info().valid())
                {
                    auto data = sample.data();
                    auto new_id = data.vehicle_id();
                    auto new_pose = data.pose();
                    int out_edge_index = -1;
                    int out_edge_path_index = -1;
                    bool matched = laneGraph.map_match_pose(new_pose, out_edge_index, out_edge_path_index);

                    if(matched)
                    {
                        vehicle_id = new_id;
                        current_edge_index = out_edge_index;
                        current_edge_path_index = out_edge_path_index;
                        std::cout << "Vehicle matched" << std::endl;
                        return;
                    }
                }
            }
        }, 
        cpm::ParticipantSingleton::Instance(), 
        cpm::get_topic<VehicleObservation>("vehicleObservation")
    );




    /*
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

    {
        auto next_edges = laneGraph.find_subsequent_edges(out_edge_index);

        std::cout << "next_edges" << std::endl;
        for(auto i:next_edges)
        {
            std::cout << i << std::endl;
        }
    }


    std::cout << "random route" << std::endl;
    for (int kk = 0; kk < 10000; ++kk)
    {
        auto next_edges = laneGraph.find_subsequent_edges(out_edge_index);
        out_edge_index = next_edges.at(rand() % next_edges.size());
        std::cout << out_edge_index << std::endl;
    }
    */
}