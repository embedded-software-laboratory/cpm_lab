
#include "lane_graph.hpp"
#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/AsyncReader.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Timer.hpp"
#include "VehicleObservation.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "lane_graph_tools.hpp"
#include <dds/pub/ddspub.hpp>
#include <iostream>

using std::vector;

struct VehicleTrajectoryPlanningState
{
    uint8_t vehicle_id = 0;
    size_t current_edge_index = 0;
    size_t current_edge_path_index = 0;
    double current_speed = 0;
    vector<size_t> current_route_edge_indices;


    VehicleCommandTrajectory get_trajectory_command(uint64_t t_now);
};

VehicleCommandTrajectory VehicleTrajectoryPlanningState::get_trajectory_command(uint64_t t_now)
{
    TrajectoryPoint trajectory_point;
    trajectory_point.t().nanoseconds(t_now + 1200000000ull);
    trajectory_point.px(laneGraphTools.edges_x.at(current_edge_index).at(current_edge_path_index));
    trajectory_point.py(laneGraphTools.edges_y.at(current_edge_index).at(current_edge_path_index));
    trajectory_point.vx(laneGraphTools.edges_cos.at(current_edge_index).at(current_edge_path_index) * current_speed);
    trajectory_point.vy(laneGraphTools.edges_sin.at(current_edge_index).at(current_edge_path_index) * current_speed);
    VehicleCommandTrajectory vehicleCommandTrajectory;
    vehicleCommandTrajectory.vehicle_id(vehicle_id);
    vehicleCommandTrajectory.trajectory_points(rti::core::vector<TrajectoryPoint>(1, trajectory_point));
    return vehicleCommandTrajectory;
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


    VehicleTrajectoryPlanningState trajectoryPlan;


    // receive vehicle pose from IPS
    cpm::AsyncReader<VehicleObservation> vehicleObservations_reader(
        [&](dds::sub::LoanedSamples<VehicleObservation>& samples) {

            if (trajectoryPlan.vehicle_id > 0) return;

            for(auto sample : samples) 
            {
                if(sample.info().valid())
                {
                    auto data = sample.data();
                    auto new_id = data.vehicle_id();
                    auto new_pose = data.pose();
                    int out_edge_index = -1;
                    int out_edge_path_index = -1;
                    bool matched = laneGraphTools.map_match_pose(new_pose, out_edge_index, out_edge_path_index);

                    if(matched)
                    {
                        trajectoryPlan.vehicle_id = new_id;
                        trajectoryPlan.current_edge_index = out_edge_index;
                        trajectoryPlan.current_edge_path_index = out_edge_path_index;
                        trajectoryPlan.current_route_edge_indices.push_back(trajectoryPlan.current_edge_index);
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
        if(trajectoryPlan.vehicle_id == 0)
        {
            std::cout << "Waiting for vehicle ..." << std::endl;
            return;
        }

        assert(trajectoryPlan.current_route_edge_indices.size() >= 1);
        assert(trajectoryPlan.current_route_edge_indices[0] == trajectoryPlan.current_edge_index);

        // extend route randomly
        while(trajectoryPlan.current_route_edge_indices.size() < 15)
        {
            auto next_edges = laneGraphTools.find_subsequent_edges(trajectoryPlan.current_route_edge_indices.back());
            assert(next_edges.size() > 0);
            trajectoryPlan.current_route_edge_indices.push_back(next_edges.at(rand() % next_edges.size()));
        }

        // longitudinal dynamics
        double acceleration = 0.6;
        if(trajectoryPlan.current_speed >= 1.2)
        {
            acceleration = 0;
        }
        const double dt = (dt_nanos*1e-9);
        trajectoryPlan.current_speed += dt * acceleration;
        const double delta_s = dt * trajectoryPlan.current_speed;



        laneGraphTools.move_along_route
        (
            trajectoryPlan.current_route_edge_indices, 
            trajectoryPlan.current_edge_index, 
            trajectoryPlan.current_edge_path_index, 
            delta_s
        );

        while(trajectoryPlan.current_route_edge_indices.at(0) != trajectoryPlan.current_edge_index)
        {
            trajectoryPlan.current_route_edge_indices.erase(trajectoryPlan.current_route_edge_indices.begin());
        }




        // Send trajectory point on DDS
        writer_vehicleCommandTrajectory.write(trajectoryPlan.get_trajectory_command(t_now));

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