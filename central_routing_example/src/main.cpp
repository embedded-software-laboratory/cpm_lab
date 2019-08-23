
#include "lane_graph.hpp"
#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/MultiVehicleReader.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Timer.hpp"
#include "VehicleObservation.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "VehicleTrajectoryPlanningState.hpp"
#include "lane_graph_tools.hpp"
#include <dds/pub/ddspub.hpp>
#include <iostream>
#include <memory>

using std::vector;


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


    std::map<uint8_t, std::shared_ptr<VehicleTrajectoryPlanningState> > trajectoryPlans;

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
                //e.second->avoid_collisions();
            }

            for(auto &e:trajectoryPlans)
            {
                writer_vehicleCommandTrajectory.write(e.second->get_trajectory_command(t_now));
                e.second->apply_timestep(dt_nanos);
                e.second->extend_random_route(15);
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
                    trajectoryPlans[new_id] = std::make_shared<VehicleTrajectoryPlanningState>(new_id, out_edge_index, out_edge_path_index);
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