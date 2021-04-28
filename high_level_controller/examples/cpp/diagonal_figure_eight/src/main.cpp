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
#include "VehicleCommandTrajectory.hpp"
#include <dds/pub/ddspub.hpp>
#include <iostream>
#include <memory>

using std::vector;

//Description for bash files
/**
 * \defgroup diagonal_figure_eight_files Additional Files
 * \ingroup diagonal_figure_eight
 */

/**
 * \page diagonal_figure_eight_files_page Additional Files for Diagonal Figure Eight
 * \subpage d_f_e_build <br>
 * \subpage d_f_e_run <br>
 * \ingroup diagonal_figure_eight_files
*/

/**
 * \page d_f_e_build build.bash
 * \brief Build script for diagonal_figure_eight
 */

/**
 * \page d_f_e_run run.bash
 * \brief Run script for diagonal_figure_eight
 */

/**
 * \brief Main function of the diagonal_figure_eight scenario
 * This tutorial is also described at https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Diagonal+Figure+Eight+Example
 * \param argc Command line param
 * \param argv Command line param
 * \ingroup diagonal_figure_eight
 */
int main(int argc, char *argv[])
{
    //Initialize cpm library
    const std::string node_id = "diagonal_figure_eight";
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


    HLCCommunicator hlc_communicator(
            vehicle_id
    );

    // Writer for sending trajectory commands, Writer writes the trajectory commands in the DDS "Cloud" so other programs can access them.
    // Instead of creating a new participant, we can just use the one created by the HLCCommunicator
    //For more information see our documentation about RTI DDS
    cpm::Writer<VehicleCommandTrajectory> writer_vehicleCommandTrajectory(
            hlc_communicator.getLocalParticipant()->get_participant(),
            "vehicleCommandTrajectory");


    // Figure eight trajectory data
    vector<double> trajectory_px        = vector<double>{           -1,             0,             1,             0};
    vector<double> trajectory_py        = vector<double>{           -1,             0,             1,             0};
    //In this figure eight the circles of the eight are not perfectly round but a little streched to show you the impact of the vector (Vx, vy)
    //at the starting point. 
    //Note that also the duration changes accordingly.
    vector<double> trajectory_vx        = vector<double>{            0.928,          -0.928,           0.928,         -0.928};
    vector<double> trajectory_vy        = vector<double>{          -0.928,         0.928,           -0.928,         0.928};
    vector<uint64_t> segment_duration = vector<uint64_t>{2116000000ull, 2116000000ull, 2116000000ull, 2116000000ull};


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
    uint64_t t_now = 0;


    // The code inside the onEachTimestep method is executed each timestep.
    // Here we assume that we send trajectories every 200ms
    // This means we need to manually set the middleware_period_ms parameter in the LCC to 200ms.
    // Commands must be sent to the vehicle regularly, more than 2x per second.
    // Otherwise it is assumed that the connection is lost and the vehicle stops.
    const uint64_t dt_nanos = 200000000ull; // 200 milliseconds == 200000000 nanoseconds

    // This code will get executed only once at the beginning of the first timestep
    hlc_communicator.onFirstTimestep([&](VehicleStateList vehicle_state_list)
    {
        // Initial time used for trajectory generation
        reference_trajectory_time = vehicle_state_list.t_now() + 1000000000ull;

        // Check if middleware_period_ms was set correctly, as described above
        // If not, write a message to log
        if( vehicle_state_list.period_ms()*1000000ull != dt_nanos ){
            cpm::Logging::Instance().write(1,
                    "middleware_period_ms needs to be set to 200ms");
        }
    });

    hlc_communicator.onEachTimestep([&](VehicleStateList vehicle_state_list)
    {
        t_now = vehicle_state_list.t_now();
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

    // This says 'We are done setting up, please let us start planning'
    hlc_communicator.start();
}
