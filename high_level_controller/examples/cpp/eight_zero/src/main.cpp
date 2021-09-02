#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Timer.hpp"
#include "cpm/Writer.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "Eight.hpp"
#include <iostream>
#include <memory>
#include <stdlib.h>



using std::vector;


//Description for bash files
/**
 * \defgroup eight_zero_files Additional Files
 * \ingroup eight_zero
 */

/**
 * \page eight_zero_files_page Additional Files for Eight Zero
 * \subpage e_z_build <br>
 * \subpage e_z_run <br>
 * \ingroup eight_zero_files
*/

/**
 * \page e_z_build build.bash
 * \brief Build script for eight_zero
 */

/**
 * \page e_z_run run.bash
 * \brief Run script for eight_zero
 */

/**
 * \brief Main function of the eight_zero scenario
 * \param argc Command line param
 * \param argv Command line param
 * \ingroup eight_zero
 */
int main(int argc, char *argv[])
{
    const std::string node_id = "eight_zero";
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id(node_id);
    const bool enable_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);
    const std::vector<int> vehicle_ids_int = cpm::cmd_parameter_ints("vehicle_ids", {4}, argc, argv);
    std::vector<uint8_t> vehicle_ids;
    for(auto i:vehicle_ids_int)
    {
        assert(i>0);
        assert(i<255);
        vehicle_ids.push_back(i);
    }
    assert(vehicle_ids.size() > 0);
    uint8_t vehicle_id = vehicle_ids.at(0);



    // Writer for sending trajectory commands
    cpm::Writer<VehicleCommandTrajectory> writer_vehicleCommandTrajectory("vehicleCommandTrajectory");


    // Initialize 8-Trajectory
    Eight eight;

    // This variabel tracks the reference state of the time,
    // it is incremented as time passes. It refers to the first trajectory point the vehicle will drive to.
    uint64_t reference_trajectory_time = 0;
    // This variable refers to the last currently planned trajectory point the vehicle will drive to.           --TODO
    uint64_t trajectory_duration = 0;

    // Saves the current trajectory which is to be sent
    vector<TrajectoryPoint> trajectory_points;

    // The code inside the cpm::Timer is executed every 400 milliseconds.
    // Commands must be sent to the vehicle regularly, more than 2x per second.
    // Otherwise it is assumed that the connection is lost and the vehicle stops.
    const uint64_t dt_nanos = 200000000ull; // 400 milliseconds == 400000000 nanoseconds
    auto timer = cpm::Timer::create(node_id, dt_nanos, 0, false, true, enable_simulated_time);
    timer->start([&](uint64_t t_now)
    {
        // Initial time used for trajectory generation
        if (reference_trajectory_time == 0) reference_trajectory_time = t_now + 2000000000ull;

        // Append new points to the trajectory
        while (reference_trajectory_time + trajectory_duration < t_now + 4000000000ull){
            TrajectoryPoint trajectory_point = eight.get_trajectoryPoint();
            trajectory_point.t().nanoseconds(reference_trajectory_time + trajectory_duration);

            trajectory_points.push_back(trajectory_point);

            trajectory_duration += eight.get_segment_duration();
            eight.move_forward(); //TODO Fitting with initial point?
        }


        // Delete outdated points, i.e., remove first point if second one lies in past, too,
        while (!trajectory_points.empty() && trajectory_points.at(1).t().nanoseconds() < t_now){
            uint64_t delta_t = trajectory_points.at(1).t().nanoseconds() - trajectory_points.at(0).t().nanoseconds();
            trajectory_duration -= delta_t;
            reference_trajectory_time += delta_t;

            trajectory_points.erase(trajectory_points.begin());
        }

        // Send the current trajectory 
        rti::core::vector<TrajectoryPoint> rti_trajectory_points(trajectory_points);
        VehicleCommandTrajectory vehicle_command_trajectory;
        vehicle_command_trajectory.vehicle_id(vehicle_id);
        vehicle_command_trajectory.trajectory_points(rti_trajectory_points);
        vehicle_command_trajectory.header().create_stamp().nanoseconds(t_now);
        vehicle_command_trajectory.header().valid_after_stamp().nanoseconds(t_now + 1000000000ull);
        writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);
    });
}