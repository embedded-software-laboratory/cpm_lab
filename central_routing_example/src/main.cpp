
#include "lane_graph.hpp" //sw-folder central routing->include
#include "cpm/Logging.hpp" //cpm_base->cpm_lib->include->cpm
#include "cpm/CommandLineReader.hpp" //cpm_base->cpm_lib->include->cpm
#include "cpm/init.hpp" //cpm_base->cpm_lib->include->cpm
#include "cpm/MultiVehicleReader.hpp" //cpm_base->cpm_lib->include->cpm
#include "cpm/ParticipantSingleton.hpp" //cpm_base->cpm_lib->include->cpm
#include "cpm/Timer.hpp" //cpm_base->cpm_lib->include->cpm
#include "VehicleObservation.hpp" 
#include "VehicleCommandTrajectory.hpp"
#include "VehicleTrajectoryPlanningState.hpp" //sw-folder central routing
#include "lane_graph_tools.hpp" //sw-folder central routing
#include <dds/pub/ddspub.hpp> //rti folder
#include <iostream>
#include <sstream>
#include <memory>
#include <mutex>
#include <thread>
#include "MultiVehicleTrajectoryPlanner.hpp" //sw-folder central routing

using std::vector;

int main(int argc, char *argv[])
{   //////////////////Set logging details///////////////////////////////////////////////////////////
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id("central_routing_example");
    const bool enable_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv); //variable is set to false 
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
    MultiVehicleTrajectoryPlanner planner(dt_nanos);


    ///////////// writer and reader for sending trajectory commands////////////////////////
    //the writer will write data for the trajectory for the position of the vehicle (x,y) and the speed for each direction vecotr (vx,vy) and the vehicle ID
    dds::pub::DataWriter<VehicleCommandTrajectory> writer_vehicleCommandTrajectory
    (
        dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
        cpm::get_topic<VehicleCommandTrajectory>("vehicleCommandTrajectory")
    );
    //the reader will read the pose of a vehicle given by its vehicle ID
    cpm::MultiVehicleReader<VehicleObservation> ips_reader(
        cpm::get_topic<VehicleObservation>("vehicleObservation"),
        vehicle_ids
    );

    /////////////////////////////////Trajectory planner//////////////////////////////////////////
    //create(node_id, period in nanoseconds, offset in nanoseconds, bool wait_for_start, bool simulated_time_allowed, bool simulated_time (set in line 27))
    auto timer = cpm::Timer::create("central_routing_example", dt_nanos, 0, false, true, enable_simulated_time); 
    timer->start([&](uint64_t t_now)
    {
        planner.set_real_time(t_now);

        if(planner.is_started())//will be set to true after fist activation
        {
            auto computation_start_time = timer->get_time();
            //get trajectory commands from MultiVehicleTrajectoryPlanner with new points for each vehicle ID
            auto commands = planner.get_trajectory_commands(t_now);
            auto computation_end_time = timer->get_time();

            cpm::Logging::Instance().write("%s, Computation start time: %llu, Computation end time: %llu", vehicle_ids_string.c_str(), computation_start_time, computation_end_time);
            for(auto& command:commands)
            {
                writer_vehicleCommandTrajectory.write(command);
            }
        }
        else //prepare to start planner
        {
            std::map<uint8_t, VehicleObservation> ips_sample;
            std::map<uint8_t, uint64_t> ips_sample_age;
            ips_reader.get_samples(t_now, ips_sample, ips_sample_age);
            //check for vehicles if online
            bool all_vehicles_online = true;
            for(auto e:ips_sample_age)
            {
                if(e.second > 1000000000ull) all_vehicles_online = false;
            }

            if(!all_vehicles_online)
            {
                std::cout << "Waiting for vehicles ..." << std::endl;
                return;
            }

            bool all_vehicles_matched = true;
            //match pose of vehicles with pose on map
            for(auto e:ips_sample)
            {
                auto data = e.second;
                auto new_id = data.vehicle_id();
                auto new_pose = data.pose();
                int out_edge_index = -1;
                int out_edge_path_index = -1;
                bool matched = laneGraphTools.map_match_pose(new_pose, out_edge_index, out_edge_path_index);
                //if vehicle was found on map, add vehicle to MultiVehicleTrajectoryPlanner
                if(matched)
                {
                    planner.add_vehicle(std::make_shared<VehicleTrajectoryPlanningState>(new_id, out_edge_index, out_edge_path_index));
                    std::cout << "Vehicle " << int(new_id) << " matched" << std::endl;
                }
                else //Errormessage, if not all vehicles could be matched to the map
                {
                    all_vehicles_matched = false;
                    std::cout << "Vehicle " << int(new_id) << " not matched" << std::endl;
                }
            }

            if(all_vehicles_matched)
            {   
                //Start the Planner. That includes collision avoidance. In this case we avoid collisions by priority assignment
                //with the consequence of speed reduction for the lower prioritized vehicle (here: Priority based on descending vehicle ID of the neighbours.)
                planner.start();
            }
        }
    });
}