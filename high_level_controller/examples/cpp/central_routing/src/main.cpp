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
#include "cpm/Writer.hpp"
#include "cpm/Participant.hpp"
#include "cpm/HLCCommunicator.hpp"
#include "VehicleStateList.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "VehicleTrajectoryPlanningState.hpp"   //sw-folder central routing
#include "lane_graph_tools.hpp"                 //sw-folder central routing
#include <iostream>
#include <sstream>
#include <memory>
#include <mutex>
#include <thread>
#include <set>
#include <stdexcept>
#include "MultiVehicleTrajectoryPlanner.hpp"    //sw-folder central routing

using std::vector;

//Description for bash files
/**
 * \defgroup central_routing_files Additional Files
 * \ingroup central_routing
 */

/**
 * \page central_routing_files_page Additional Files for Central Routing
 * \subpage central_routing_build <br>
 * \subpage central_routing_run <br>
 * \ingroup central_routing_files
*/

/**
 * \page central_routing_build build.bash
 * \brief Build script for central_routing
 */

/**
 * \page central_routing_run run.bash
 * \brief Run script for central_routing
 */

/**
 * \brief Main function of the central_routing scenario
 * This tutorial is also described at https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Central+Routing+Example
 * \ingroup central_routing
 */
int main(int argc, char *argv[])
{   //////////////////Set logging details///////////////////////////////////////////////////////////
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id("central_routing");
    ////////////////Set vehicle IDs for the vehicles selected in the command line or the LCC////////
    const std::vector<int> vehicle_ids_int = cpm::cmd_parameter_ints("vehicle_ids", {4}, argc, argv);
    std::vector<uint8_t> vehicle_ids;
    for(auto i:vehicle_ids_int)
    {
        assert(i>0);
        assert(i<255);
        vehicle_ids.push_back(i);
    }

    ////////////////Outstream in shell which vehicles were selected/////////////////////////////////
    std::stringstream vehicle_ids_stream;
    vehicle_ids_stream << "Vehicle IDs: ";
    for (uint8_t id : vehicle_ids)
    {
        vehicle_ids_stream << static_cast<uint32_t>(id) << "|"; //Cast s.t. uint8_t is not interpreted as a character
    }
    std::string vehicle_ids_string = vehicle_ids_stream.str();

    std::cout << vehicle_ids_string << std::endl;

    //////////////Initialization for trajectory planning/////////////////////////////////
    // Definition of a timesegment in nano seconds and a trajecotry planner for more than one vehicle
    const uint64_t dt_nanos = 400000000ull;
    // MultiVehicleTrajectoryPlanner planner(dt_nanos);
    std::unique_ptr<MultiVehicleTrajectoryPlanner> planner = std::unique_ptr<MultiVehicleTrajectoryPlanner>(new MultiVehicleTrajectoryPlanner(dt_nanos));


    HLCCommunicator hlc_communicator(vehicle_ids);


    ///////////// writer and reader for sending trajectory commands////////////////////////
    //the writer will write data for the trajectory for the position of the vehicle (x,y) and the speed for each direction vecotr (vx,vy) and the vehicle ID
    cpm::Writer<VehicleCommandTrajectory> writer_vehicleCommandTrajectory(
            hlc_communicator.getLocalParticipant()->get_participant(), 
            "vehicleCommandTrajectory");

    /////////////////////////////////Trajectory planner//////////////////////////////////////////
    hlc_communicator.onFirstTimestep([&](VehicleStateList vehicle_state_list) {
            // reset planner object
            planner = std::unique_ptr<MultiVehicleTrajectoryPlanner>(new MultiVehicleTrajectoryPlanner(dt_nanos));
            planner->set_real_time(vehicle_state_list.t_now());

            //check for vehicles if online
            bool all_vehicles_online = true;
            std::set<uint8_t> vehicle_checklist(vehicle_ids.begin(), vehicle_ids.end());
            for(auto vehicle_state:vehicle_state_list.state_list())
            {
                if(vehicle_checklist.find(vehicle_state.vehicle_id()) != vehicle_checklist.end()) {
                    vehicle_checklist.erase(vehicle_state.vehicle_id());
                }
            }

            all_vehicles_online = (vehicle_checklist.size() == 0);

            if(!all_vehicles_online)
            {
                cpm::Logging::Instance().write(
                    1,
                    "Error: Cannot start planning, can't find all vehicles in VehicleStateList from middleware"
                );
                throw std::runtime_error("Can't find all vehicles in VehicleStateList");
            }

            bool all_vehicles_matched = true;
            //match pose of vehicles with pose on map
            for(auto vehicle_state:vehicle_state_list.state_list())
            {
                auto new_id = vehicle_state.vehicle_id();
                auto new_pose = vehicle_state.pose();
                int out_edge_index = -1;
                int out_edge_path_index = -1;
                bool matched = laneGraphTools.map_match_pose(new_pose, out_edge_index, out_edge_path_index);
                //if vehicle was found on map, add vehicle to MultiVehicleTrajectoryPlanner
                if(matched)
                {
                    planner->add_vehicle(std::make_shared<VehicleTrajectoryPlanningState>(new_id, out_edge_index, out_edge_path_index));
                    cpm::Logging::Instance().write(
                        3,
                        "Vehicle %d matched.",
                        int(new_id)
                    );
                }
                else //Errormessage, if not all vehicles could be matched to the map
                {
                    all_vehicles_matched = false;
                    cpm::Logging::Instance().write(
                        1,
                        "Error: Cannot start planning, vehicle %d not matched.",
                        int(new_id)
                    );
                    throw std::runtime_error("Couldn't match vehicle, see logs for details");
                }
            }

            if(all_vehicles_matched)
            {   
                //Start the Planner. That includes collision avoidance. In this case we avoid collisions by priority assignment
                //with the consequence of speed reduction for the lower prioritized vehicle (here: Priority based on descending vehicle ID of the neighbours.)
                planner->start();
            }
            else {
                cpm::Logging::Instance().write(
                    1,
                    "Error: Cannot start planning, couldn't find all vehicles' positions"
                );
                throw std::runtime_error("Couldn't find all vehicles' positions");
            }
    });
    hlc_communicator.onEachTimestep([&](VehicleStateList vehicle_state_list) {
            // Do not start if middleware period is not 400ms
            // because it's required by this planner
            if(vehicle_state_list.period_ms()*1000000 != dt_nanos) {
                cpm::Logging::Instance().write(1,
                    "Please set middleware_period_ms to 400ms"
                );
                return;
            }

            uint64_t t_now = vehicle_state_list.t_now();
            planner->set_real_time(t_now);

            if(planner->is_started())//will be set to true after fist activation
            {
                //get trajectory commands from MultiVehicleTrajectoryPlanner with new points for each vehicle ID
                auto commands = planner->get_trajectory_commands(t_now);
                
                for(auto& command:commands)
                {
                    writer_vehicleCommandTrajectory.write(command);
                }
            }
            else //prepare to start planner
            {
                //Something went wrong
            }
    });

    hlc_communicator.start();
}
