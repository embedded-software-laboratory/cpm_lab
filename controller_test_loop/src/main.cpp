#include <iostream>
#include <algorithm>
#include <thread>
#include <map>
#include <stdlib.h>
#include <unistd.h>
#include <cmath>
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

class TrajectoryIndex
{
private:
    //This map keeps track of the current trajectory index of each vehicle
    //Do not access these two 
    std::mutex trajectory_mutex;
    std::map<uint8_t, int64_t> vehicle_trajectory_indices;

    //Reference to required data
    std::vector<TrajectoryPoint>& trajectory_points;
    std::mutex& observation_mutex;
    std::map<uint8_t, VehicleObservation>& vehicle_observations;

    double get_squared_distance(double x_1, double y_1, double x_2, double y_2)
    {
        return (x_1 - x_2) * (x_1 - x_2) 
            + (y_1 - y_2) * (y_1 - y_2);
    }

    bool has_collision(size_t next_index, uint8_t vehicle_id)
    {
        TrajectoryPoint next_pose = trajectory_points.at(next_index);
        size_t next_other_index;
        bool has_seen_own_id = false;

        for (const auto& entry : vehicle_trajectory_indices)
        {
            //Only handle collisions for IDs that come before the own ID
            if (entry.first == vehicle_id)
            {
                has_seen_own_id = true;
            }

            //Id comes before own id: Let that vehicle drive first. Else: Do not crash into current position of other vehicle
            if (!has_seen_own_id)
            {
                next_other_index = (entry.second + 1) % (trajectory_points.size());
            }
            else {
                next_other_index = entry.second;
            }

            if(get_squared_distance(next_pose.px(), next_pose.py(), trajectory_points.at(next_other_index).px(), trajectory_points.at(next_other_index).py())
                < 0.07)
            {
                return true;
            }
        }

        return false;
    }

public:
    //Get references to data required for the computation
    TrajectoryIndex(std::vector<TrajectoryPoint>& _trajectory_points, std::mutex& _observation_mutex, std::map<uint8_t, VehicleObservation>& _vehicle_observations) :
        trajectory_points(_trajectory_points),
        observation_mutex(_observation_mutex),
        vehicle_observations(_vehicle_observations)
    {

    }

    void set_closest_trajectory_point(uint8_t id)
    {
        //Lock map so that it cannot be changed elsewhere, then get trajectory index
        std::lock_guard<std::mutex> lock(observation_mutex);
        std::lock_guard<std::mutex> lock2(trajectory_mutex);

        //Only continue if the position of the vehicle is known
        if (vehicle_observations.find(id) != vehicle_observations.end())
        {
            Pose2D vehicle_position = vehicle_observations.at(id).pose();
            double distance = 0;
            double smallest_distance = 0xffffffffffffffffull;
            size_t smallest_index = 0;

            //Find closest trajectory point
            for (size_t index = 0; index < trajectory_points.size(); ++index)
            {
                //Avoid taking the square root, too expensive and not necessary
                distance = get_squared_distance(trajectory_points.at(index).px(), trajectory_points.at(index).py(), vehicle_position.x(), vehicle_position.y());

                if(distance < smallest_distance)
                {
                    smallest_distance = distance;
                    smallest_index = index;
                }
            }

            //Store its index - get one point back to make sure that the vehicle will 'catch' the trajectory properly
            if (smallest_index > 0)
            {
                vehicle_trajectory_indices[id] = smallest_index - 1;
            }
            else
            {
                vehicle_trajectory_indices[id] = trajectory_points.size() - 1;
            }
        }
    }

    /**
     * \brief Returns the trajectory point for the given id OR tries to calculate one - this might fail, so no valid value is returned in that case
     */
    int64_t get_next_trajectory_index(uint8_t id)
    {
        if (vehicle_trajectory_indices.find(id) != vehicle_trajectory_indices.end())
        {
            //Increment trajectory index
            int64_t old_point = vehicle_trajectory_indices.at(id);
            int64_t next_point = (old_point + 1)  % (trajectory_points.size());

            if (has_collision(next_point, id))
            {
                vehicle_trajectory_indices[id] = old_point;
            }
            else
            {
                vehicle_trajectory_indices[id] = next_point;
            }

            return old_point;
        }
        else
        {
            set_closest_trajectory_point(id);
            return -1;
        }
    }
};

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


    std::mutex observation_mutex;
    std::mutex slot_mutex;

    // There are 'n_max_vehicles' slots for vehicles in the reference trajectory.
    // If the slot ID is zero, the slot is unused.
    std::vector<uint8_t> slot_vehicle_ids(n_max_vehicles, 0);

    // Receive IPS DDS data
    std::map<uint8_t, VehicleObservation> vehicleObservations;

    // vehicles without a slot
    std::vector<uint8_t> unassigned_vehicle_ids;

    //Object to retrieve current trajectory index for each vehicle
    TrajectoryIndex trajectory_index(trajectory_points, observation_mutex, vehicleObservations);

    // receive vehicle pose from IPS
    cpm::AsyncReader<VehicleObservation> vehicleObservations_reader(
        [&](dds::sub::LoanedSamples<VehicleObservation>& samples) {
            std::unique_lock<std::mutex> lock(observation_mutex);
            std::unique_lock<std::mutex> lock2(slot_mutex);
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
    uint64_t offset = 0;
    if (enable_simulated_time) {
        offset = 1000000000ull;
    }
    auto timer = cpm::Timer::create("controller_test_loop", point_period_nanoseconds, offset, false, true, enable_simulated_time);
    timer->start([&](uint64_t t_now) {
        std::unique_lock<std::mutex> lock(slot_mutex);

        //Vehicles should be taken from their current position, the starting point of the trajectory should not just be anywhere depending on the current time
        //Thus, a first-time init is required to find out where the vehicles are, and to set corresponding initial indices for each vehicle
        //A similar thing is also done when a trajectory for a new vehicle must be created
        static bool position_init = false;
        if (!position_init)
        {
            //Find closest point in trajectory for each vehicle
            //Store this initial index for the vehicle
            for (const auto& id : slot_vehicle_ids)
            {
                if (id != 0)
                {
                    trajectory_index.set_closest_trajectory_point(id);
                }
            }

            position_init = true;
        }

        const uint64_t t_eval = t_now + 1000000000ull;


        for (size_t slot_idx = 0; slot_idx < slot_vehicle_ids.size(); ++slot_idx)
        {
            const uint8_t vehicle_id = slot_vehicle_ids.at(slot_idx);

            // slot is assigned, just send the trajectory
            if(vehicle_id > 0)
            {
                int64_t index = trajectory_index.get_next_trajectory_index(vehicle_id);
                if (index >= 0)
                {
                    auto trajectory_point = trajectory_points.at(static_cast<size_t>(index));
                    trajectory_point.t().nanoseconds(t_eval);

                    // Send trajectory point on DDS
                    VehicleCommandTrajectory vehicleCommandTrajectory;
                    vehicleCommandTrajectory.vehicle_id(vehicle_id);
                    vehicleCommandTrajectory.trajectory_points(rti::core::vector<TrajectoryPoint>(1, trajectory_point));

                    writer_vehicleCommandTrajectory.write(vehicleCommandTrajectory);
                }
            }
            // Slot is unassigned, maybe it can be matched with an unassigned vehicle
            else
            {
                for (size_t i = 0; i < unassigned_vehicle_ids.size(); ++i)
                {
                    const uint8_t unassigned_vehicle_id = unassigned_vehicle_ids[i];
                    if (unassigned_vehicle_id == 0) continue;

                    //Try again to get the trajectory index
                    int64_t index = trajectory_index.get_next_trajectory_index(unassigned_vehicle_id);

                    //Then try to work with that data
                    std::lock_guard<std::mutex> lock(observation_mutex);

                    if (index >= 0)
                    {
                        auto trajectory_point = trajectory_points.at(static_cast<size_t>(index));
                        trajectory_point.t().nanoseconds(t_eval);

                        const auto &observation = vehicleObservations[unassigned_vehicle_id];

                        const double ref_yaw = atan2(trajectory_point.vy(), trajectory_point.vx());

                        // if(fabs(trajectory_point.px() - observation.pose().x()) < 0.3
                        // && fabs(trajectory_point.py() - observation.pose().y()) < 0.3
                        // && fabs(sin(0.5*(ref_yaw - observation.pose().yaw()))) < 0.3)
                        // {
                            for (size_t k = 0; k < slot_vehicle_ids.size(); ++k)
                            {
                                assert(slot_vehicle_ids[k] != unassigned_vehicle_id);
                            }


                            slot_vehicle_ids[slot_idx] = unassigned_vehicle_id;
                            unassigned_vehicle_ids[i] = 0;
                            break;
                        //}
                    }
                }
            }
        }
        unassigned_vehicle_ids.clear();
    });

    return 0;
}