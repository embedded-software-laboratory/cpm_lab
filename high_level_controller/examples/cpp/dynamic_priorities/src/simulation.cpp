#include <iostream>
#include <vector>
#include <string> 
#include "VehicleTrajectoryPlanner.hpp"
#include "VehicleState.hpp"
#include "Pose2D.hpp"
#include "lane_graph_tools.hpp"
#include "cpm/Logging.hpp"                      //->cpm_lib->include->cpm
#include "cpm/CommandLineReader.hpp"            //->cpm_lib->include->cpm
#include "cpm/init.hpp"                         //->cpm_lib->include->cpm
#include "cpm/ParticipantSingleton.hpp"         //->cpm_lib->include->cpm
#include "cpm/Timer.hpp"                        //->cpm_lib->include->cpm
#include "cpm/get_topic.hpp"
#include "cpm/Writer.hpp"
#include "cpm/ReaderAbstract.hpp"
#include "cpm/Reader.hpp"
#include "cpm/HLCCommunicator.hpp"
#include "cpm/Participant.hpp"
#include <atomic>
#include<thread>

void simulate(std::shared_ptr<VehicleTrajectoryPlanner> planner);

// allows for syncing the hlcs that will run multithreaded
std::atomic<int> iterations;

int n_vehicles;
int n_steps;

// vehicles always plan for dt segments (in ns) 
uint64_t dt = 400000000;

// start poses on the commonroad map (central routing map)
std::vector<Pose2D>  start_poses( {Pose2D(3.15802,3.88862,-0.0507011), Pose2D(3.8791,3.72123,-0.497375), 
                                    Pose2D(4.35882,2.95562,-1.41334), Pose2D(4.39471,1.99965,-1.56801), 
                                    Pose2D(4.35932,1.04568,-1.72724), Pose2D(3.87964,0.278448,-2.64031), 
                                    Pose2D(3.15654,0.111178,-3.09329), Pose2D(2.24999,0.104926,3.14091),
                                    Pose2D(1.34368,0.111556,3.08998), Pose2D(0.62071,0.278691,2.63872),
                                    Pose2D(0.141397,1.04581,1.7287), Pose2D(0.104006,2.00092,1.57371),
                                    Pose2D(0.141131,2.9547,1.4113), Pose2D(0.62021,3.72161,0.500138),
                                    Pose2D(1.34238,3.88924,0.0494666), Pose2D(3.05031,2.22542,3.14112),
                                    Pose2D(3.05088,1.77524,-0.00189507), Pose2D(2.47512,1.19944,1.57269),
                                    Pose2D(2.02503,1.19963,-1.57305), Pose2D(1.4504,1.77449,-0.0020518)});

/*
*   Simulates a vehicle for n_steps. Synced to the other vehicle via iterations.
*/
void simulate(std::shared_ptr<VehicleTrajectoryPlanner> planner){
    uint64_t t = 10; // initial time !=0
    while(iterations < n_vehicles * n_steps)
    {
        if (iterations % n_vehicles == 0)
        {
            planner->plan(t, dt);
            t +=dt;
            std::cout << "TIME: " << t << std::endl;
            iterations++;
        }
    }
    
}

/*
*   Allows for the dynamic_priorities hlc to be simulated at some interval for a given amount of steps (each step covers dt).
*   In modes Static, Random, FCA: (0,1,2)
*   For n vehicles.
*   (logs will be written by the planner instances)
*/
int main(int argc, char* argv[])
{
    std::vector<std::string> arguments = {"dds_domain=21", "dds_initial_peer=rtps@udpv4://192.168.1.249:25598"};

    n_vehicles = cpm::cmd_parameter_int("n", 5, argc, argv);
    const PriorityMode mode = static_cast<PriorityMode>(
        cpm::cmd_parameter_int(
            "hlc_mode", static_cast<int>(PriorityMode::fca), argc, argv
        )
    );
    n_steps = cpm::cmd_parameter_int("steps", 450, argc, argv);

    // create vehicle planners
    std::vector<std::shared_ptr<VehicleTrajectoryPlanner>> vehicles;
    std::vector<uint8_t> vec;
    uint8_t id = 0;
    for (int i = 0; i < n_vehicles; i++)
    {
        vehicles.push_back(std::move(std::shared_ptr<VehicleTrajectoryPlanner>(new VehicleTrajectoryPlanner(mode))));
        cpm::Logging::Instance().set_id("dynamic_priorities");
        id ++;
        vec.push_back(id);
    }

    // setup planners
    uint8_t vehicle_id = 0;
    for (auto&& planner : vehicles)
    {
        vehicle_id++;
        // Writer to communicate plans with other vehicles
        cpm::Writer<Trajectory> writer_trajectory(
            "trajectory");
        // Reader to receive planned trajectories of other vehicles
        cpm::ReaderAbstract<Trajectory> reader_trajectory(
            "trajectory");
        
        planner->set_writer(
            std::unique_ptr<cpm::Writer<Trajectory>>(
                new cpm::Writer<Trajectory>( "trajectory")));
        planner->set_reader(
            std::unique_ptr<cpm::ReaderAbstract<Trajectory>>(
                new cpm::ReaderAbstract<Trajectory>( "trajectory")));
        // set fca reader and writer
        planner->set_fca_reader(
            std::unique_ptr<cpm::ReaderAbstract<FutureCollisionAssessment>>(
                new cpm::ReaderAbstract<FutureCollisionAssessment>("futureCollisionAssessment")));
        planner->set_fca_writer(
            std::unique_ptr<cpm::Writer<FutureCollisionAssessment>>(
                new cpm::Writer<FutureCollisionAssessment>("futureCollisionAssessment")));

        // Writer to send visualization to middleware
        planner->set_visualization_writer(
            std::unique_ptr<cpm::Writer<Visualization>>(
                new cpm::Writer<Visualization>(
                    "visualization")));


        // sets the vehicles up at the starting poses, mathes them to the nodes of the map graph (include/lane_graph)
        Pose2D pose = start_poses.at(vehicle_id - 1); // ids start at 1; start_poses at 0

        int out_edge_index = -1;
        int out_edge_path_index = -1;
        bool matched = false;
        matched = laneGraphTools.map_match_pose(pose, out_edge_index, out_edge_path_index);
        if (!matched)
        {
        }
        else
        {
            CouplingGraph coupling_graph(vec);

            planner->set_coupling_graph(coupling_graph);
            // Initialize PlanningState with starting position
            planner->set_vehicle(
                std::unique_ptr<VehicleTrajectoryPlanningState>(
                    new VehicleTrajectoryPlanningState(
                        vehicle_id,
                        out_edge_index,
                        out_edge_path_index,
                        dt
                    )
                )
            );
        }
    }

    // creates a thread for each vehicle, it lives for the duration of the simulation
    std::vector<std::thread> threads;
    threads.reserve(n_vehicles);
    for (auto planner : vehicles)
    {
        threads.emplace_back(simulate, planner);
    }
    // wait for the threads to finish
    for (auto &thread : threads)
    {
        thread.join();
    }
}