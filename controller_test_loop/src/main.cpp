#include <iostream>
#include <algorithm>
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
#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"



int main(int argc, char *argv[])
{
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id("controller_test_loop");
    const bool enable_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);

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


    std::mutex _mutex;

    // There are 'n_max_vehicles' slots for vehicles in the reference trajectory.
    // If the slot ID is zero, the slot is unused.
    std::vector<uint8_t> slot_vehicle_ids(n_max_vehicles, 0);

    // Receive IPS DDS data
    std::map<uint8_t, VehicleObservation> vehicleObservations;

    // vehicles without a slot
    std::vector<uint8_t> unassigned_vehicle_ids;

    // receive vehicle pose from IPS
    cpm::AsyncReader<VehicleObservation> vehicleObservations_reader(
        [&](dds::sub::LoanedSamples<VehicleObservation>& samples) {
            std::unique_lock<std::mutex> lock(_mutex);
            for(auto sample : samples) 
            {
                if(sample.info().valid())
                {
                    auto data = sample.data();
                    vehicleObservations[data.vehicle_id()] = data;

                    bool vehicle_has_slot = false;
                    for (size_t slot_idx = 0; slot_idx < slot_vehicle_ids.size(); ++slot_idx)
                    {
                        if(data.vehicle_id() > 0 && slot_vehicle_ids[slot_idx] == data.vehicle_id())
                        {
                            vehicle_has_slot = true;
                            break;
                        }
                    }
                    if(!vehicle_has_slot) 
                    {
                        if(std::find(unassigned_vehicle_ids.begin(), unassigned_vehicle_ids.end(), data.vehicle_id()) 
                            == unassigned_vehicle_ids.end()) 
                        {
                            unassigned_vehicle_ids.push_back(data.vehicle_id());
                        }
                    }
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




    // Control loop, which sends trajectory commands to the vehicles
    auto timer = cpm::Timer::create("controller_test_loop", 40000000ull, 0, false, true, enable_simulated_time);
    timer->start([&](uint64_t t_now) {
        std::unique_lock<std::mutex> lock(_mutex);

        const uint64_t t_eval = ((t_now + 1000000000ull) / point_period_nanoseconds) * point_period_nanoseconds;


        for (size_t slot_idx = 0; slot_idx < slot_vehicle_ids.size(); ++slot_idx)
        {
            const uint8_t vehicle_id = slot_vehicle_ids.at(slot_idx);

            uint64_t trajectory_index = 
                (t_eval + slot_idx * vehicle_time_gap_nanoseconds) / point_period_nanoseconds;

            trajectory_index = trajectory_index % trajectory_points.size();

            auto trajectory_point = trajectory_points.at(trajectory_index);
            trajectory_point.t().nanoseconds(t_eval);


            // slot is assigned, just send the trajectory
            if(vehicle_id > 0)
            {
                // Send trajectory point on DDS
                VehicleCommandTrajectory vehicleCommandTrajectory;
                vehicleCommandTrajectory.vehicle_id(vehicle_id);
                vehicleCommandTrajectory.trajectory_points(rti::core::vector<TrajectoryPoint>(1, trajectory_point));
                writer_vehicleCommandTrajectory.write(vehicleCommandTrajectory);
            }
            // Slot is unassigned, maybe it can be matched with an unassigned vehicle
            else
            {
                for (size_t i = 0; i < unassigned_vehicle_ids.size(); ++i)
                {
                    const uint8_t unassigned_vehicle_id = unassigned_vehicle_ids[i];

                    if (unassigned_vehicle_id == 0) continue;

                    const auto &observation = vehicleObservations[unassigned_vehicle_id];

                    const double ref_yaw = atan2(trajectory_point.vy(), trajectory_point.vx());

                    if(fabs(trajectory_point.px() - observation.pose().x()) < 0.3
                    && fabs(trajectory_point.py() - observation.pose().y()) < 0.3
                    && fabs(sin(0.5*(ref_yaw - observation.pose().yaw()))) < 0.3)
                    {
                        for (size_t k = 0; k < slot_vehicle_ids.size(); ++k)
                        {
                            assert(slot_vehicle_ids[k] != unassigned_vehicle_id);
                        }


                        slot_vehicle_ids[slot_idx] = unassigned_vehicle_id;
                        unassigned_vehicle_ids[i] = 0;
                        break;
                    }
                }
            }
        }
        unassigned_vehicle_ids.clear();
    });

    return 0;
}