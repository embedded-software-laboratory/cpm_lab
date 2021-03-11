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
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Timer.hpp"
#include "cpm/get_time_ns.hpp"
#include "cpm/Writer.hpp"
#include "VehicleCommandTrajectory.hpp"
#include <iostream>
#include <memory>

using std::vector;

//Description for bash files
/**
 * \defgroup basic_line_files Additional Files
 * \ingroup basic_line
 */

/**
 * \page basic_line_files_page Additional Files for Basic Line
 * \subpage basic_line_build <br>
 * \subpage basic_line_run <br>
 * \ingroup basic_line_files
*/

/**
 * \page basic_line_build build.bash
 * \brief Build script for basic_line
 */

/**
 * \page basic_line_run run.bash
 * \brief Run script for basic_line
 */

/**
 * \brief Main function of the basic_line scenario
 * Read this before you start:
 *     Vehicle Commands: http://cpm-lab.embedded.rwth-aachen.de:8090/display/CLD/Vehicle+Commands
 *     Timer:            http://cpm-lab.embedded.rwth-aachen.de:8090/pages/viewpage.action?pageId=2293786
 * \ingroup basic_line
 */
int main(int argc, char *argv[])
{
    const std::string node_id = "basic_line";
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

  
    vector<double> trajectory_px        = vector<double>{-0.5, 0,  0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
    vector<double> trajectory_py        = vector<double>{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    vector<double> trajectory_vx        = vector<double>{0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    vector<double> trajectory_vy        = vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    vector<uint64_t> segment_duration = vector<uint64_t>{1000000000ull, 1000000000ull, 1000000000ull, 1000000000ull, 1000000000ull, 1000000000ull, 1000000000ull, 1000000000ull, 1000000000ull, 1000000000ull};



    assert(segment_duration.size() == trajectory_px.size());
    assert(segment_duration.size() == trajectory_py.size());
    assert(segment_duration.size() == trajectory_vx.size());
    assert(segment_duration.size() == trajectory_vy.size());

    const double map_center_x = 0.25;
    const double map_center_y = 2.0;
    for (double &px : trajectory_px)
    {
        px += map_center_x;
    }
    for (double &py : trajectory_py)
    {
        py += map_center_y;
    }

    auto t_now = cpm::get_time_ns();
    uint64_t reference_trajectory_time = t_now + 10000000000ull;
    size_t reference_trajectory_index = 0;

    //Vector of trajectory points (send whole circle for interpolation, always with the current point as second point to interpolate to)
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
    vehicle_command_trajectory.header().valid_after_stamp().nanoseconds(reference_trajectory_time);
    
    for (int i = 0; i < 10; ++i)
    {
        writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);
        sleep(1);
    }

    std::cout << "Sent: " << vehicle_command_trajectory << std::endl;

    sleep(10);
}