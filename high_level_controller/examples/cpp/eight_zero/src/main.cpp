// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/HLCCommunicator.hpp"
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


    // Ease-of-life class to communicate with the middleware.
    // This participant should only communicate on this system, so its messages are not directly sent to the vehicles.
    // Instead we communicate with the middleware, and the middleware relays these messages to the vehicle.
    // These settings are saved in the Quality of Service (QoS) xml-file and are identical to the ones the middleware uses.
    // One QoS file can define multiple profiles, which is why we need to specify that we want to use the
    // LocalCommunicationProfile, from the MatlabLibrary.
    HLCCommunicator hlc_communicator(
            vehicle_id
    );
    
    // Writer for sending trajectory commands
    cpm::Writer<VehicleCommandTrajectory> writer_vehicleCommandTrajectory(
            hlc_communicator.getLocalParticipant()->get_participant(),
            "vehicleCommandTrajectory");


    // Initialize 8-Trajectory
    Eight eight;

    // This variabel tracks the reference state of the time,
    // it is incremented as time passes. It refers to the first trajectory point the vehicle will drive to.
    uint64_t reference_trajectory_time = 0;
    // This variable refers to the last currently planned trajectory point the vehicle will drive to.           --TODO
    uint64_t trajectory_duration = 0;
    uint64_t t_now = 0;

    // Saves the current trajectory which is to be sent
    vector<TrajectoryPoint> trajectory_points;

    // The code inside the onEachTimestep method is executed each timestep.
    // Here we assume that we send trajectories every 200ms
    // This means we need to manually set the middleware_period_ms parameter in the LCC to 200ms.
    // Commands must be sent to the vehicle regularly, more than 2x per second.
    // Otherwise it is assumed that the connection is lost and the vehicle stops.
    const uint64_t dt_nanos = 200000000ull; // 400 milliseconds == 400000000 nanoseconds

    hlc_communicator.onFirstTimestep([&](VehicleStateList vehicle_state_list)
    {
        // Initial time used for trajectory generation
        reference_trajectory_time = vehicle_state_list.t_now() + 2000000000ull;

        // Check if middleware_period_ms was set correctly, as described above
        // If not, write a message to log
        if( vehicle_state_list.period_ms()*1000000ull != dt_nanos ){
            cpm::Logging::Instance().write(1,
                    "middleware_period_ms needs to be set to 200ms");
        }
    });

    // The code inside the cpm::Timer is executed every 400 milliseconds.
    // Commands must be sent to the vehicle regularly, more than 2x per second.
    // Otherwise it is assumed that the connection is lost and the vehicle stops.
    hlc_communicator.onEachTimestep([&](VehicleStateList vehicle_state_list)
    {
        t_now = vehicle_state_list.t_now();

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

    hlc_communicator.start();
}
