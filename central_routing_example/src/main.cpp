
#include "lane_graph.hpp"
#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/AsyncReader.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Timer.hpp"
#include "VehicleObservation.hpp"
#include "VehicleCommandTrajectory.hpp"
#include <dds/pub/ddspub.hpp>
#include <iostream>

using std::vector;

class LaneGraphTools : public LaneGraph
{
    std::vector<std::vector<double>> edges_s;
public:
    LaneGraphTools();
    bool map_match_pose(Pose2D pose, int &out_edge_index, int &out_edge_path_index);
    vector<size_t> find_subsequent_edges(int edge_index);
    void move_along_route(vector<size_t> route_edge_indices, size_t &edge_index, size_t &edge_path_index, double delta_s);
};

LaneGraphTools::LaneGraphTools()
{
    // Compute path arc lengths
    for (size_t i_edge = 0; i_edge < n_edges; ++i_edge)
    {
        std::vector<double> edge_s;
        edge_s.push_back(0);

        for (size_t i_path = 1; i_path < n_edge_path_nodes; ++i_path)
        {
            const double dx = edges_x[i_edge][i_path] - edges_x[i_edge][i_path-1];
            const double dy = edges_y[i_edge][i_path] - edges_y[i_edge][i_path-1];
            const double len = sqrt(dx*dx+dy*dy);
            edge_s.push_back(edge_s.back() + len);
        }
        assert(edge_s.size() == edges_x[i_edge].size());
        edges_s.push_back(edge_s);
    }
}

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

void LaneGraphTools::move_along_route(
    vector<size_t> route_edge_indices, 
    size_t &edge_index, 
    size_t &edge_path_index, 
    double delta_s)
{
    assert(route_edge_indices.size() > 2);
    assert(route_edge_indices[0] == edge_index);
    assert(delta_s >= 0);

    size_t route_index = 0;

    while(1)
    {
        // Do we need to go to the next edge?
        const double remaining_distance_current_edge = 
            edges_s[edge_index][n_edge_path_nodes-1] - edges_s[edge_index][edge_path_index];

        if(remaining_distance_current_edge > delta_s)
        {
            // We are on the final edge
            // Advance path nodes until delta_s is zero
            while(1)
            {
                if(edge_path_index >= n_edge_path_nodes-1) break;
                const double step_length = edges_s[edge_index][edge_path_index+1] - edges_s[edge_index][edge_path_index];
                if(delta_s < step_length) break;
                delta_s -= step_length;
                edge_path_index++;
            }
            return;
        }
        else
        {
            // Need to go to the next edge
            delta_s -= remaining_distance_current_edge;
            route_index++;
            assert(route_index < route_edge_indices.size());
            edge_index = route_edge_indices.at(route_index);
            edge_path_index = 0;
        }

        /*std::cout <<
        "  route_index " << route_index <<
        "  remaining_distance_current_edge " << remaining_distance_current_edge <<
        "  delta_s " << delta_s <<
        "  edge_index " << edge_index <<
        "  edge_path_index " << edge_path_index <<
        std::endl;*/
    }
}

int main(int argc, char *argv[])
{
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id("central_routing_example");
    const bool enable_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);


    // Writer for sending trajectory commands
    dds::pub::DataWriter<VehicleCommandTrajectory> writer_vehicleCommandTrajectory
    (
        dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
        cpm::get_topic<VehicleCommandTrajectory>("vehicleCommandTrajectory")
    );



    uint8_t vehicle_id = 0;
    size_t current_edge_index = 0;
    size_t current_edge_path_index = 0;
    double current_speed = 0;
    vector<size_t> current_route_edge_indices;


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
                        current_route_edge_indices.push_back(current_edge_index);
                        std::cout << "Vehicle matched" << std::endl;
                        return;
                    }
                }
            }
        }, 
        cpm::ParticipantSingleton::Instance(), 
        cpm::get_topic<VehicleObservation>("vehicleObservation")
    );


    const uint64_t dt_nanos = 400000000ull;
    auto timer = cpm::Timer::create("central_routing_example", dt_nanos, 0, false, true, enable_simulated_time);
    timer->start([&](uint64_t t_now) 
    {
        if(vehicle_id == 0)
        {
            std::cout << "Waiting for vehicle ..." << std::endl;
            return;
        }

        assert(current_route_edge_indices.size() >= 1);
        assert(current_route_edge_indices[0] == current_edge_index);

        // extend route randomly
        while(current_route_edge_indices.size() < 15)
        {
            auto next_edges = laneGraph.find_subsequent_edges(current_route_edge_indices.back());
            assert(next_edges.size() > 0);
            current_route_edge_indices.push_back(next_edges.at(rand() % next_edges.size()));
        }

        // longitudinal dynamics
        double acceleration = 0.2;
        if(current_speed >= 1.5)
        {
            acceleration = 0;
        }
        const double dt = (dt_nanos*1e-9);
        current_speed += dt * acceleration;
        const double delta_s = dt * current_speed;



        laneGraph.move_along_route
        (
            current_route_edge_indices, 
            current_edge_index, 
            current_edge_path_index, 
            delta_s
        );

        while(current_route_edge_indices.at(0) != current_edge_index)
        {
            current_route_edge_indices.erase(current_route_edge_indices.begin());
        }

        /*std::cout << "current_speed " << current_speed << std::endl;
        std::cout << "current_edge_index " << current_edge_index << std::endl;
        std::cout << "current_edge_path_index " << current_edge_path_index << std::endl;*/


        // Send trajectory point on DDS
        TrajectoryPoint trajectory_point;
        trajectory_point.t().nanoseconds(t_now + 1200000000ull);
        trajectory_point.px(laneGraph.edges_x.at(current_edge_index).at(current_edge_path_index));
        trajectory_point.py(laneGraph.edges_y.at(current_edge_index).at(current_edge_path_index));
        trajectory_point.vx(laneGraph.edges_cos.at(current_edge_index).at(current_edge_path_index) * current_speed);
        trajectory_point.vy(laneGraph.edges_sin.at(current_edge_index).at(current_edge_path_index) * current_speed);
        VehicleCommandTrajectory vehicleCommandTrajectory;
        vehicleCommandTrajectory.vehicle_id(vehicle_id);
        vehicleCommandTrajectory.trajectory_points(rti::core::vector<TrajectoryPoint>(1, trajectory_point));
        writer_vehicleCommandTrajectory.write(vehicleCommandTrajectory);

    });



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