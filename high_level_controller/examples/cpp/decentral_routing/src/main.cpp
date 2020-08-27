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


#include "lane_graph.hpp"                       //sw-folder central routing->include
#include "cpm/Logging.hpp"                      //->cpm_lib->include->cpm
#include "cpm/CommandLineReader.hpp"            //->cpm_lib->include->cpm
#include "cpm/init.hpp"                         //->cpm_lib->include->cpm
#include "cpm/ParticipantSingleton.hpp"         //->cpm_lib->include->cpm
#include "cpm/Timer.hpp"                        //->cpm_lib->include->cpm
#include "VehicleObservation.hpp" 
#include "VehicleCommandTrajectory.hpp"
#include "lane_graph_tools.hpp"                 //sw-folder central routing
#include <dds/pub/ddspub.hpp>                   //rti folder
#include <iostream>
#include <sstream>
#include <memory>
#include <mutex>
#include <thread>
#include "VehicleTrajectoryPlanner.hpp"    //sw-folder central routing

using std::vector;

int main(int argc, char *argv[])
{   //////////////////Set logging details///////////////////////////////////////////////////////////
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id("central_routing");
    const bool enable_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv); //variable is set to false 
    ////////////////Set vehicle IDs for the vehicles selected in the command line or the LCC////////
    const std::vector<int> vehicle_ids_int = cpm::cmd_parameter_ints("vehicle_ids", {4}, argc, argv);
    uint8_t vehicle_id;

    assert(vehicle_ids_int.size()==1);

    for(auto i:vehicle_ids_int)
    {
        assert(i>0);
        assert(i<255);
        vehicle_id = i;
    }

    ////////////////Outstream in shell which vehicles were selected/////////////////////////////////
    std::stringstream vehicle_id_stream;
    vehicle_id_stream << "Started HLC for Vehicle ID: ";
    vehicle_id_stream << static_cast<uint32_t>(vehicle_id) << "|"; //Cast s.t. uint8_t is not interpreted as a character
    std::string vehicle_id_string = vehicle_id_stream.str();

    std::cout << vehicle_id_string << std::endl;

    //////////////Initialization for trajectory planning/////////////////////////////////
    // Definition of a timesegment in nano seconds and a trajecotry planner for more than one vehicle
    const uint64_t dt_nanos = 400000000ull;
    // VehicleTrajectoryPlanner planner(dt_nanos);
    std::unique_ptr<VehicleTrajectoryPlanner> planner = std::unique_ptr<VehicleTrajectoryPlanner>(new VehicleTrajectoryPlanner(dt_nanos));


    ///////////// writer and reader for sending trajectory commands////////////////////////
    //the writer will write data for the trajectory for the position of the vehicle (x,y) and the speed for each direction vecotr (vx,vy) and the vehicle ID
    dds::pub::DataWriter<VehicleCommandTrajectory> writer_vehicleCommandTrajectory
    (
        dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
        cpm::get_topic<VehicleCommandTrajectory>("vehicleCommandTrajectory")
    );
    //the reader will read the pose of a vehicle given by its vehicle ID
    //cpm::Reader<VehicleObservation> ips_reader(
    //    cpm::get_topic<VehicleObservation>("vehicleObservation"),
    //    vehicle_id
    //);
    
    // These readers and writers are used to exchange planned trajectories
    // between the VehicleTrajectoryPlanners
    auto writer_laneGraphTrajectory = std::make_shared< dds::pub::DataWriter<LaneGraphTrajectory> >(
        dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
        cpm::get_topic<LaneGraphTrajectory>("laneGraphTrajectory")
    );
    auto reader_laneGraphTrajectory = std::make_shared< dds::sub::DataReader<LaneGraphTrajectory> >(
        dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), 
        cpm::get_topic<LaneGraphTrajectory>("laneGraphTrajectory")
    );

    /////////////////////////////////Trajectory planner//////////////////////////////////////////
    //create(node_id, period in nanoseconds, offset in nanoseconds, bool wait_for_start, bool simulated_time_allowed, bool simulated_time (set in line 27))
    auto timer = cpm::Timer::create("decentral_routing", dt_nanos, 0, false, true, enable_simulated_time); 
    timer->start([&](uint64_t t_now)
    {
        planner->set_real_time(t_now);

        if(planner->is_started())//will be set to true after fist activation
        {
            auto computation_start_time = timer->get_time();
            //get trajectory commands from VehicleTrajectoryPlanner with new points
            auto command = planner->get_trajectory_command(t_now);
            auto computation_end_time = timer->get_time();

            cpm::Logging::Instance().write(
                3,
                "%s, Computation start time: %llu, Computation end time: %llu",
                vehicle_id_string.c_str(), computation_start_time, computation_end_time
            );
            
            writer_vehicleCommandTrajectory.write(command);
        }
        else //prepare to start planner
        {
            // reset planner object
            planner = std::unique_ptr<VehicleTrajectoryPlanner>(new VehicleTrajectoryPlanner(dt_nanos));
            planner->set_real_time(t_now);

            planner->set_vehicle(std::make_shared<VehicleTrajectoryPlanningState>(vehicle_id, -1, -1));
            }

            planner->set_writer(writer_laneGraphTrajectory);

            planner->set_reader(reader_laneGraphTrajectory);

            //TODO: Check if vehicles are ready like central_routing does

            //Start the Planner. That includes collision avoidance. In this case we avoid collisions by priority assignment
            //with the consequence of speed reduction for the lower prioritized vehicle (here: Priority based on descending vehicle ID of the neighbours.)
            planner->start();
    });
}
