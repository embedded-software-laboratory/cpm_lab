
#include "lane_graph.hpp"
#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/MultiVehicleReader.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Timer.hpp"
#include "VehicleObservation.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "lane_graph_tools.hpp"
#include <dds/pub/ddspub.hpp>
#include <iostream>

using std::vector;

class VehicleTrajectoryPlanningState
{
    uint8_t vehicle_id = 0;
    size_t current_edge_index = 0;
    size_t current_edge_path_index = 0;
    double current_speed = 0;
    vector<size_t> current_route_edge_indices;

    void invariant();
public:

    VehicleTrajectoryPlanningState(){}
    VehicleTrajectoryPlanningState(
        uint8_t _vehicle_id,
        size_t _edge_index,
        size_t _edge_path_index);

    VehicleCommandTrajectory get_trajectory_command(uint64_t t_now);
    void extend_random_route(size_t n);
    void apply_timestep(uint64_t dt_nanos);
};

VehicleTrajectoryPlanningState::VehicleTrajectoryPlanningState(
    uint8_t _vehicle_id,
    size_t _edge_index,
    size_t _edge_path_index)
:vehicle_id(_vehicle_id)
,current_edge_index(_edge_index)
,current_edge_path_index(_edge_path_index)
,current_route_edge_indices({_edge_index})
{

}

void VehicleTrajectoryPlanningState::invariant()
{
    assert(current_route_edge_indices.size() >= 1);
    assert(current_route_edge_indices[0] == current_edge_index);
    assert(current_edge_index < laneGraphTools.n_edges);
    assert(current_edge_path_index < laneGraphTools.n_edge_path_nodes);
}

void VehicleTrajectoryPlanningState::apply_timestep(uint64_t dt_nanos)
{
    double acceleration = 0.6;
    if(current_speed >= 1.2)
    {
        acceleration = 0;
    }
    const double dt = (dt_nanos*1e-9);
    current_speed += dt * acceleration;
    const double delta_s = dt * current_speed;

    laneGraphTools.move_along_route
    (
        current_route_edge_indices, 
        current_edge_index, 
        current_edge_path_index, 
        delta_s
    );

    while( !current_route_edge_indices.empty()
        && current_route_edge_indices.at(0) != current_edge_index)
    {
        current_route_edge_indices.erase(current_route_edge_indices.begin());
    }

    invariant();
}

void VehicleTrajectoryPlanningState::extend_random_route(size_t n)
{
    invariant();

    while(current_route_edge_indices.size() < n)
    {
        auto next_edges = laneGraphTools.find_subsequent_edges(current_route_edge_indices.back());
        assert(next_edges.size() > 0);
        current_route_edge_indices.push_back(next_edges.at(rand() % next_edges.size()));
    }
}

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
    const std::vector<int> vehicle_ids_int = cpm::cmd_parameter_ints("vehicle_ids", {4}, argc, argv);
    std::vector<uint8_t> vehicle_ids;
    for(auto i:vehicle_ids_int)
    {
        assert(i>0);
        assert(i<255);
        vehicle_ids.push_back(i);
    }


    std::map<uint8_t, VehicleTrajectoryPlanningState> trajectoryPlans;

    // Writer for sending trajectory commands
    dds::pub::DataWriter<VehicleCommandTrajectory> writer_vehicleCommandTrajectory
    (
        dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
        cpm::get_topic<VehicleCommandTrajectory>("vehicleCommandTrajectory")
    );

    cpm::MultiVehicleReader<VehicleObservation> ips_reader(
        cpm::get_topic<VehicleObservation>("vehicleObservation"),
        vehicle_ids
    );

    bool started = false;

    const uint64_t dt_nanos = 400000000ull;
    auto timer = cpm::Timer::create("central_routing_example", dt_nanos, 0, false, true, enable_simulated_time);
    timer->start([&](uint64_t t_now) 
    {
        if(started)
        {
            for(auto &e:trajectoryPlans)
            {
                auto &trajectoryPlan = e.second;
                trajectoryPlan.extend_random_route(15);
                trajectoryPlan.apply_timestep(dt_nanos);

                // Send trajectory point on DDS
                writer_vehicleCommandTrajectory.write(trajectoryPlan.get_trajectory_command(t_now));
            }
        }
        else
        {
            std::map<uint8_t, VehicleObservation> ips_sample;
            std::map<uint8_t, uint64_t> ips_sample_age;
            ips_reader.get_samples(t_now, ips_sample, ips_sample_age);

            bool all_vehicles_online = true;
            for(auto e:ips_sample_age)
            {
                if(e.second > 1000000000ull) all_vehicles_online = false;
            }

            if(!all_vehicles_online)
            {
                std::cout << "Waiting for vehicles ..." << std::endl;
                return;
            }

            bool all_vehicles_matched = true;

            for(auto e:ips_sample)
            {
                auto data = e.second;
                auto new_id = data.vehicle_id();
                auto new_pose = data.pose();
                int out_edge_index = -1;
                int out_edge_path_index = -1;
                bool matched = laneGraphTools.map_match_pose(new_pose, out_edge_index, out_edge_path_index);

                if(matched)
                {
                    trajectoryPlans[new_id] = VehicleTrajectoryPlanningState(new_id, out_edge_index, out_edge_path_index);
                    std::cout << "Vehicle " << int(new_id) << " matched" << std::endl;
                }
                else
                {
                    all_vehicles_matched = false;
                    std::cout << "Vehicle " << int(new_id) << " not matched" << std::endl;
                }
            }

            if(all_vehicles_matched)
            {
                started = true;
            }
        }
    });
}