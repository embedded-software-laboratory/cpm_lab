#include <iostream>
#include <thread>
#include <map>
#include <stdlib.h>
#include <unistd.h>
#include "cpm/AsyncReader.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Timer.hpp"
#include "cpm/get_topic.hpp"
#include "VehicleObservation.hpp"
#include "test_loop_trajectory.hpp"



int main(int argc, char* argv[])
{
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
 

    // Control loop, which sends commands to the vehicles
    auto timer = cpm::Timer::create("controller_test_loop", 40000000ull, 0, false, true);
    timer->start([&](uint64_t t_now) {
        std::unique_lock<std::mutex> lock(vehicleObservations_mutex);

        const uint64_t loop_count = t_now / loop_period_nanoseconds;
        const uint64_t loop_offset_time = t_now % loop_period_nanoseconds;
        const uint64_t loop_start_time = loop_count * loop_period_nanoseconds;

        trajectory_index == (t_now + slot_id * vehicle_time_gap_nanoseconds) / point_period_nanoseconds
    });

    return 0;
}