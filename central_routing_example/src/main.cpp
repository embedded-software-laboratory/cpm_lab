
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
#include <mutex>
#include <thread>

using std::vector;

struct MultiVehicleTrajectoryPlanner
{
    std::map<uint8_t, std::shared_ptr<VehicleTrajectoryPlanningState> > trajectoryPlans;
    bool started = false;
    uint64_t t_start = 0;
    uint64_t t_real_time = 0;
    std::mutex mutex;

    std::map<uint8_t, std::vector<TrajectoryPoint> > trajectory_point_buffer;

};


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

    MultiVehicleTrajectoryPlanner planner;


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



    const uint64_t dt_nanos = 400000000ull;


    std::thread planning_thread([&](){
        while(!planner.started) usleep(110000);


        uint64_t t_planning = 0;

        {
            std::lock_guard<std::mutex> lock(planner.mutex); 
            t_planning = planner.t_real_time;
        }

        while(1)
        {
            // Priority based collision avoidance: Every vehicle avoids 
            // the 'previous' vehicles, i.e. those with a smaller ID.
            vector< std::shared_ptr<VehicleTrajectoryPlanningState> > previous_vehicles;
            for(auto &e:planner.trajectoryPlans)
            {
                e.second->avoid_collisions(previous_vehicles);
                previous_vehicles.push_back(e.second);
            }

            {
                std::lock_guard<std::mutex> lock(planner.mutex); 
                for(auto &e:planner.trajectoryPlans)
                {
                    while(planner.trajectory_point_buffer[e.first].size() > 9)
                    {
                        planner.trajectory_point_buffer[e.first].erase(planner.trajectory_point_buffer[e.first].begin());
                    }
                    auto trajectory_point = e.second->get_trajectory_point();
                    trajectory_point.t().nanoseconds(trajectory_point.t().nanoseconds() + planner.t_start);
                    planner.trajectory_point_buffer[e.first].push_back(trajectory_point);
                }
            }

            for(auto &e:planner.trajectoryPlans)
            {
                e.second->apply_timestep(dt_nanos);
            }

            t_planning += dt_nanos;

            while(planner.t_real_time + 6000000000ull < t_planning) usleep(110000);
        }
    });


    auto timer = cpm::Timer::create("central_routing_example", dt_nanos, 0, false, true, enable_simulated_time);
    timer->start([&](uint64_t t_now)
    {
        std::lock_guard<std::mutex> lock(planner.mutex); 
        planner.t_real_time = t_now;

        if(planner.started)
        {
            for(auto &e:planner.trajectory_point_buffer)
            {
                VehicleCommandTrajectory vehicleCommandTrajectory;
                vehicleCommandTrajectory.vehicle_id(e.first);
                vehicleCommandTrajectory.trajectory_points(rti::core::vector<TrajectoryPoint>(e.second));
                writer_vehicleCommandTrajectory.write(vehicleCommandTrajectory);
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
                    planner.trajectoryPlans[new_id] = std::make_shared<VehicleTrajectoryPlanningState>(new_id, out_edge_index, out_edge_path_index);
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
                planner.started = true;
                planner.t_start = t_now + 2000000000ull;
            }
        }
    });
}