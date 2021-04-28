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

#include <iostream>
#include <memory>

#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/Participant.hpp"
#include "cpm/Writer.hpp"
#include "cpm/HLCCommunicator.hpp"

// Import the DDS message types we will use
#include "VehicleCommandTrajectory.hpp"
#include "VehicleStateList.hpp"
#include "ReadyStatus.hpp"

using std::vector;

//Description for bash files
/**
 * \defgroup basic_circle_files Additional Files
 * \ingroup basic_circle
 */

/**
 * \page basic_circle_files_page Additional Files for Basic Circle
 * \subpage basic_circle_build <br>
 * \subpage basic_circle_run <br>
 * \ingroup basic_circle_files
*/

/**
 * \page basic_circle_build build.bash
 * \brief Build script for basic_circle
 */

/**
 * \page basic_circle_run run.bash
 * \brief Run script for basic_circle
 */

/**
 * \brief Main function of the basic_circle scenario
 * This tutorial is also described in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Basic+Circle+Example
 * \ingroup basic_circle
 */
int main(int argc, char *argv[])
{
    //Initialize cpm library
    const std::string node_id = "basic_circle";
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

    //Give user time to drag-and-drop the vehicle to the right position in simulation
    //(The vehicle does not consider 'mixed' trajectories as before, when trajectories were sent e.g. by both the LCC and the program)
    sleep(10);

    // Ease-of-life class to communicate with the middleware.
    // This participant should only communicate on this system, so its messages are not directly sent to the vehicles.
    // Instead we communicate with the middleware, and the middleware relays these messages to the vehicle.
    // These settings are saved in the Quality of Service (QoS) xml-file and are identical to the ones the middleware uses.
    // One QoS file can define multiple profiles, which is why we need to specify that we want to use the
    // LocalCommunicationProfile, from the MatlabLibrary.
    HLCCommunicator hlc_communicator(
            vehicle_id
    );

    // Writer for sending trajectory commands, Writer writes the trajectory commands in the DDS "Cloud" so other programs can access them.
    // Instead of creating a new participant, we can just use the one created by the HLCCommunicator
    //For more information see our documentation about RTI DDS
    cpm::Writer<VehicleCommandTrajectory> writer_vehicleCommandTrajectory(
            hlc_communicator.getLocalParticipant()->get_participant(),
            "vehicleCommandTrajectory");

    // Circle trajectory data
    //In this section the points on the x and y axis (independently from the map!) are set. 
    //They are relative to the defined center point defined below as map_center_x and map_center_y
    vector<double> trajectory_px        = vector<double>{            1,             0,            -1,             0};
    vector<double> trajectory_py        = vector<double>{            0,             1,             0,            -1};
    //These vecotrs define the speed in x and y direction. Together the define the starting direction from the current trajectory point,
    // for example: vx = 1 and vy = 1 will lead to a positive diagonal starting vector from the starting point. For more informations see
    //our documentation website
    // For these we assume that we send trajectories every 200ms
    // This means we need to manually set the middleware_period_ms parameter in the LCC to 200ms.
    vector<double> trajectory_vx        = vector<double>{            0,            -1,             0,             1};
    vector<double> trajectory_vy        = vector<double>{            1,             0,            -1,             0};
    vector<uint64_t> segment_duration = vector<uint64_t>{1570800000ull, 1570800000ull, 1570800000ull, 1570800000ull};

    /*
    // Figure eight trajectory data
    vector<double> trajectory_px        = vector<double>{           -1,             0,             1,             0};
    vector<double> trajectory_py        = vector<double>{            0,             0,             0,             0};
    //In this figure eight the circles of the eight are not perfectly round but a little streched to show you the impact of the vector (Vx, vy)
    //at the starting point. 
    //Note that also the duration changes accordingly.
    vector<double> trajectory_vx        = vector<double>{            0,          0.14,             0,         -0.14};
    vector<double> trajectory_vy        = vector<double>{          1.3,         -1.27,           1.3,         -1.27};
    vector<uint64_t> segment_duration = vector<uint64_t>{1700000000ull, 1700000000ull, 1700000000ull, 1700000000ull};
    */

    assert(segment_duration.size() == trajectory_px.size());
    assert(segment_duration.size() == trajectory_py.size());
    assert(segment_duration.size() == trajectory_vx.size());
    assert(segment_duration.size() == trajectory_vy.size());

    //Definition of the center point for the circle and the figure eight
    const double map_center_x = 2.25;
    const double map_center_y = 2.0;
    for (double &px : trajectory_px)
    {
        px += map_center_x;
    }
    for (double &py : trajectory_py)
    {
        py += map_center_y;
    }


    // These variabels track the reference state,
    // they are incremented as time passes.
    int reference_trajectory_index = 0;
    uint64_t reference_trajectory_time = 0;


    // The code inside onEachTimestep is executed every each timestep sent by the middleware.
    // We assume it to be 200ms here.
    // Commands must be sent to the vehicle regularly, more than 2x per second.
    // Otherwise it is assumed that the connection is lost and the vehicle stops.
    hlc_communicator.onEachTimestep([&](VehicleStateList vehicle_state_list)
    {
        uint64_t t_now = vehicle_state_list.t_now();
        // Initial time used for trajectory generation
        if (reference_trajectory_time == 0) reference_trajectory_time = t_now + 1000000000ull;


        vector<TrajectoryPoint> trajectory_points;
        for (size_t i = 0; i < segment_duration.size(); ++i)
        {
            size_t trajectory_index = (reference_trajectory_index + i) % segment_duration.size();
            uint64_t trajectory_time = reference_trajectory_time + (i - 1) * segment_duration[reference_trajectory_index];

            TrajectoryPoint trajectory_point;
            trajectory_point.px(trajectory_px[trajectory_index]);
            trajectory_point.py(trajectory_py[trajectory_index]);
            trajectory_point.vx(trajectory_vx[trajectory_index]);
            trajectory_point.vy(trajectory_vy[trajectory_index]);
            trajectory_point.t().nanoseconds(trajectory_time); //This needs to be improved in case the durations are different

            trajectory_points.push_back(trajectory_point);
        }

        // Send the current trajectory 
        rti::core::vector<TrajectoryPoint> rti_trajectory_points(trajectory_points);
        VehicleCommandTrajectory vehicle_command_trajectory;
        vehicle_command_trajectory.vehicle_id(vehicle_id);
        vehicle_command_trajectory.trajectory_points(rti_trajectory_points);
        vehicle_command_trajectory.header().create_stamp().nanoseconds(t_now);
        vehicle_command_trajectory.header().valid_after_stamp().nanoseconds(t_now + 1000000000ull);
        writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);

        // Advance the reference state to T+1sec.
        // The reference state must be in the future,
        // to allow some time for the vehicle to receive
        // the message and anticipate the next turn.
        // We repeat the current message until the segment_duration is lower than the passed time
        while(reference_trajectory_time + segment_duration[reference_trajectory_index] < t_now + 1000000000ull)
        {
            reference_trajectory_time += segment_duration[reference_trajectory_index];
            reference_trajectory_index = (reference_trajectory_index + 1) % segment_duration.size();
        }

    });

    hlc_communicator.start();
}
