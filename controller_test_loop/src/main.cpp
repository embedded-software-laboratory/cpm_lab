#include <iostream>
#include <thread>
#include <map>
#include <stdlib.h>
#include <unistd.h>
#include <dds/pub/ddspub.hpp>
#include "cpm/AsyncReader.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Timer.hpp"
#include "cpm/get_topic.hpp"
#include "VehicleObservation.hpp"
#include "test_loop_trajectory.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "cpm/stamp_message.hpp"
#include "cpm/Logging.hpp"



int main()
{
    cpm::Logging::Instance().set_id("controller_test_loop");

    // Receive IPS DDS data
    std::map<uint8_t, VehicleObservation> vehicleObservations;
    std::mutex vehicleObservations_mutex;

    cpm::AsyncReader<VehicleObservation> vehicleObservations_reader(
        [&](dds::sub::LoanedSamples<VehicleObservation>& samples) {
            std::unique_lock<std::mutex> lock(vehicleObservations_mutex);
            for(auto sample : samples) 
            {
                if(sample.info().valid())
                {
                    auto data = sample.data();
                    vehicleObservations[data.vehicle_id()] = data;
                }
            }
        }, 
        cpm::ParticipantSingleton::Instance(), 
        cpm::get_topic<VehicleObservation>("vehicleObservation")
    );


    // Writer for sending trajectory commands
    dds::pub::DataWriter<VehicleCommandTrajectory> writer_vehicleCommandTrajectory
    (
        dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
        cpm::get_topic<VehicleCommandTrajectory>("vehicleCommandTrajectory")
    );


    // Load trajectory data
    std::vector<TrajectoryPoint> trajectory_points;
    uint64_t loop_period_nanoseconds;
    uint64_t point_period_nanoseconds;
    uint64_t vehicle_time_gap_nanoseconds;
    int n_max_vehicles;

    get_test_loop_trajectory(
        trajectory_points,
        loop_period_nanoseconds,
        point_period_nanoseconds,
        vehicle_time_gap_nanoseconds,
        n_max_vehicles
    );

    // There are 'n_max_vehicles' slots for vehicles in the reference trajectory.
    // If the slot ID is zero, the slot is unused.
    std::vector<uint8_t> slot_vehicle_ids(n_max_vehicles, 0);


    // TODO better, dynamic slot assignment
    for (int i = 0; i < n_max_vehicles; ++i)
    {
        slot_vehicle_ids[i] = i+1;
    }



    // Control loop, which sends trajectory commands to the vehicles
    auto timer = cpm::Timer::create("controller_test_loop", 40000000ull, 0, false, true);
    timer->start([&](uint64_t t_now) {
        std::unique_lock<std::mutex> lock(vehicleObservations_mutex);

        const uint64_t t_eval = ((t_now + 500000000ull) / point_period_nanoseconds) * point_period_nanoseconds;


        for (size_t slot_idx = 0; slot_idx < slot_vehicle_ids.size(); ++slot_idx)
        {
            const uint8_t vehicle_id = slot_vehicle_ids.at(slot_idx);

            if(vehicle_id > 0)
            {
                uint64_t trajectory_index = 
                    (t_eval + slot_idx * vehicle_time_gap_nanoseconds) / point_period_nanoseconds;

                trajectory_index = trajectory_index % trajectory_points.size();

                auto trajectory_point = trajectory_points.at(trajectory_index);
                trajectory_point.t().nanoseconds(t_eval);

                // Send trajectory point on DDS
                VehicleCommandTrajectory vehicleCommandTrajectory;
                vehicleCommandTrajectory.vehicle_id(vehicle_id);
                vehicleCommandTrajectory.trajectory_points(rti::core::vector<TrajectoryPoint>(1, trajectory_point));
                cpm::stamp_message(vehicleCommandTrajectory, t_now, 0);
                writer_vehicleCommandTrajectory.write(vehicleCommandTrajectory);
            }
        }

    });

    return 0;
}