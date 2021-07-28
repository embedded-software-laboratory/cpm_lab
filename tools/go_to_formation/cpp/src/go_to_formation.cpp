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

//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Feb-2021 11:34:31
//

// Include Files
#include "../include/go_to_formation.hpp"

int go_to_formation(int argc, char *argv[])
{

    //Initialize cpm library
    const std::string node_id = "go_to_formation";
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id(node_id);
    const bool enable_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);
    const std::vector<int> vehicle_ids_int = cpm::cmd_parameter_ints("vehicle_ids", {1}, argc, argv);
    std::vector<uint8_t> vehicle_ids;
    for (auto i : vehicle_ids_int)
    {
        assert(i > 0);
        assert(i < 255);
        vehicle_ids.push_back(i);
    }
    assert(vehicle_ids.size() > 0);

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
    
    ///////////// writer and reader for sending trajectory commands////////////////////////
    //the writer will write data for the trajectory for the position of the vehicle (x,y) and the speed for each direction vecotr (vx,vy) and the vehicle ID
    cpm::Writer<VehicleCommandTrajectory> writer_vehicleCommandTrajectory("vehicleCommandTrajectory");
    //the reader will read the pose of a vehicle given by its vehicle ID
    cpm::MultiVehicleReader<VehicleObservation> ips_reader(
        cpm::get_topic<VehicleObservation>("vehicleObservation"),
        vehicle_ids
    );


    ////////////////// Initialization of variables needed as arguments by matlab generated function/////////
    double vehicleIdList_data[256];
    int vehicleIdList_size[2];
    mgen::struct0_T vehiclePoses_data[256];
    int vehiclePoses_size[2];
    vehiclePoses_size[1] = 1;
    coder::array<mgen::struct1_T, 1U> generated_trajectory;

    // Driving speed of vehicles [m/s]
    const double speed = 1.0; 
    
    argInit_1xd256_real_T(vehicleIdList_data, vehicleIdList_size);
    argInit_1xd256_struct0_T(vehiclePoses_data, vehiclePoses_size);

    for (int i = 0; i < 20; i++) {
        vehicleIdList_data[i] = i+1;
    }
    vehicleIdList_size[1] = 20;


    ////////////////////// Setting of goal poses////////////////////////////
    vector<mgen::Pose2D> goal_poses;
    // Take goal poses from command line arguments
    vector<double> goal_poses_x = cpm::cmd_parameter_doubles("x", {0.0}, argc, argv);
    vector<double> goal_poses_y = cpm::cmd_parameter_doubles("y", {0.0}, argc, argv);
    vector<double> goal_poses_yaw = cpm::cmd_parameter_doubles("yaw", {0.0}, argc, argv);

    // TODO: More sophisticated check of cmd args needed
    // If goal poses have been provided via cmd args, take these. Otherwise take home poses at upper left coner of lab.
    if(goal_poses_x.size() == 1)
    {
        set_home_poses(20, goal_poses);
    }
    else if(
        goal_poses_x.size() == vehicle_ids.size() &&
        goal_poses_y.size() == vehicle_ids.size() &&
        goal_poses_yaw.size() == vehicle_ids.size()
    )
    {
        set_goal_poses_from_argc(goal_poses, goal_poses_x, goal_poses_y, goal_poses_yaw);
    }
    else
    {
        std::cout << "Poses passed as command line arguments are not valid.";
        return 1;
    }
    

    ////////////////////Initialization of variables needed for main behaviour//////////////////
    // Flags
    bool planning_started = false;
    bool trajectory_is_active = false;
    boolean_T is_path_valid; // Used by matlab generated function
    bool stop_now = false;

    // Time tracking
    uint64_t reference_time;
    uint64_t invalid_after_stamp;

    // Vehicle tracking
    vector<uint8_t> active_vehicle_ids = vehicle_ids; // Vector of vehicles that have to be positioned
    vector<uint8_t> pending_vehicle_ids; // Vector of vehicles, that could not be positioned at last try
    vector<uint8_t> old_pending_vehicle_ids; // Vector of vehicles that could not be positioned at forelast try
    
    int ego_vehicle_id = 1;
    vector<TrajectoryPoint> trajectory_points;


    //////////////Go to formation trajectory planning///////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////
    auto timer = cpm::Timer::create("go_to_formation", dt_nanos, 0, false, true, enable_simulated_time); 
    timer->start([&](uint64_t t_now)
    {

        // At first timeout set 't_now' as reference time
        if(! planning_started)
        {
            reference_time = t_now;
            invalid_after_stamp = reference_time;
            planning_started = true;
        }

        // Check if lastly sent trajectory is still valid
        if(t_now >= invalid_after_stamp){
            trajectory_is_active = false;
        }

        
        // If no valid trajectory present check termination criterion and calculate new trajectory if necessary
        if(! trajectory_is_active){

            // Check if there are still vehicles to be positioned.
            if(active_vehicle_ids.empty()){

                if(pending_vehicle_ids.empty()){

                    std::cout << "All vehicles in position. Stopping timer." << std::endl;
                    timer->stop();
                    stop_now = true;
                }
                else if(pending_vehicle_ids == old_pending_vehicle_ids){

                    std::cout << "All vehicles in position or unmovable. Stopping timer." << std::endl;
                    timer->stop();
                    stop_now = true;
                }
                else
                {
                    // There are still vehicles to be positioned, which were not movable the last time,
                    // e.g. because they were blocked by other vehicles.
                    active_vehicle_ids = pending_vehicle_ids;
                    old_pending_vehicle_ids = pending_vehicle_ids;
                    pending_vehicle_ids.clear();
                }
            }
    
            // Generate a new trajectory
            if(! stop_now){
            
                ego_vehicle_id = active_vehicle_ids[0];
                generated_trajectory.clear();

                std::map<uint8_t, VehicleObservation> ips_sample;
                std::map<uint8_t, uint64_t> ips_sample_age;
                ips_reader.get_samples(t_now, ips_sample, ips_sample_age);
                
                sample_to_matlab_type(ips_sample, vehiclePoses_data, vehicle_ids);

                // Find index of ego vehicle in vehicle_ids to choose 'goal_pose' by this index
                int index = find_veh_index(vehicle_ids, ego_vehicle_id);

                // Call trajectroy planning function
                // If a valid trajectory could be planned, 'is_path_valid' = true
                mgen::planTrajectory(
                    vehicleIdList_data,
                    vehicleIdList_size,
                    vehiclePoses_data,
                    vehiclePoses_size,
                    &goal_poses[index],
                    ego_vehicle_id, speed,
                    generated_trajectory,
                    &is_path_valid
                );
                
                auto len = *(generated_trajectory).size();
                std::cout << "path validity: " << (is_path_valid == 1) << std::endl;
            
                // If calculated trajectory is valid, turn into DDS formatted vector of trajectory points
                if(is_path_valid){
                        
                    trajectory_points.clear();

                    for (int i = 0; i < len; ++i)
                    {
                        TrajectoryPoint trajectory_point;
                        trajectory_point.px(generated_trajectory[i].px);
                        trajectory_point.py(generated_trajectory[i].py);
                        trajectory_point.vx(generated_trajectory[i].vx);
                        trajectory_point.vy(generated_trajectory[i].vy);
                        trajectory_point.t().nanoseconds(generated_trajectory[i].t + t_now);
                        trajectory_points.push_back(trajectory_point);
                    } 
                    // Define timestamp, until which current trajectory should be sent
                    invalid_after_stamp = trajectory_points.back().t().nanoseconds() + 2000000000ull;
                                                                                    
                    std::cout << "invalid after: " << invalid_after_stamp << std::endl;
                    trajectory_is_active = true;
                    active_vehicle_ids.erase(active_vehicle_ids.begin());
                }
                // If no valid path was generated, remember vehicle to try again later
                else{
                    trajectory_is_active = false;
                    pending_vehicle_ids.push_back(active_vehicle_ids[0]);
                    active_vehicle_ids.erase(active_vehicle_ids.begin());
                }
                
            }

        }

        // If there is a currently valid trajectory, send it again.
        if(trajectory_is_active){

            rti::core::vector<TrajectoryPoint> rti_trajectory_points(trajectory_points);
            VehicleCommandTrajectory vehicle_command_trajectory;
            vehicle_command_trajectory.vehicle_id(ego_vehicle_id);
            vehicle_command_trajectory.trajectory_points(rti_trajectory_points);
            vehicle_command_trajectory.header().create_stamp().nanoseconds(t_now);
            vehicle_command_trajectory.header().valid_after_stamp().nanoseconds(t_now);
            writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);

        } 
        
        
    });

    std::cout << "Finished" << std::endl;
    // Terminate the Matlab application.
    mgen::planTrajectory_terminate();
    return 0;
}


// Calculation of vector of vehicle poses at upper left corner of laboratory map for picking them up.
void set_home_poses(int n_vehicles,  vector<mgen::Pose2D> &home_poses)
{ 
  mgen::Pose2D home_pose = argInit_Pose2D();
  
  const double MAP_Y = 4.0;

  // Vehicle dimensions [m]
  const double  VEHICLE_REAR_OVERHANG = 0.03;
  const double  VEHICLE_LENGTH = 0.2200;
  const double  VEHICLE_WIDTH = 0.1070;
  
  // Arrangement of vehicles
  int max_no_vehicles = 20;
  int total_rows = 3;
  int total_columns = ceil(max_no_vehicles / total_rows);
  const double CLEARANCE = VEHICLE_WIDTH; //safety distance vehicle to vehicle and vehicle to map edge [m]
  
  int column;
  int row;

  for (int i = 1; i < n_vehicles+1; i++) {
      
      if (i % total_columns == 0)
      {
        column = total_columns; 
        row = floor(i / total_columns);
      }
      else
      {
        column = i % total_columns;
        row = 1 + floor(i / total_columns);
      }

      home_pose.x = 2 * VEHICLE_WIDTH + column * CLEARANCE + (column-1) * VEHICLE_WIDTH ;
      home_pose.y = MAP_Y - (VEHICLE_LENGTH - VEHICLE_REAR_OVERHANG) - row * CLEARANCE - (row-1) * VEHICLE_LENGTH * 2 ;
      home_pose.yaw = 90;
      home_poses.push_back(home_pose);
    }
}


// Setting vehicle goal poses from cmd line args (alternative to home poses)
void set_goal_poses_from_argc(vector<mgen::Pose2D> &poses, vector<double> &x, vector<double> &y, vector<double> &yaw){
  
  mgen::Pose2D pose = argInit_Pose2D();

  for(uint8_t i = 0; i < x.size(); i++)
  {
    pose.x = x[i];
    pose.y = y[i];
    pose.yaw = yaw[i];
    poses.push_back(pose);
  }

}

// Convert sample taken from vehicle observation into type required by matlab generated function 'planTrajectroy'
void sample_to_matlab_type(
    std::map<uint8_t, VehicleObservation> &sample,
    mgen::struct0_T (&vehiclePoses)[256],
    std::vector<uint8_t> &vehicle_ids
)
{
    vector<std::pair<int, Pose2D>> help_vector;
    int new_id;
    Pose2D new_pose;

    // Transformation of sample into format needed by matlab generated function                
    for(auto i = sample.begin(); i != sample.end(); i++)
    {
        auto data = i->second;
        help_vector.emplace_back(data.vehicle_id(), data.pose());
    }

    for(uint8_t i = 0; i < vehicle_ids.size(); i++)
    {
        new_id = help_vector[i].first;
        new_pose = help_vector[i].second;

        // Conversion from rad[-pi,pi] to deg [0, 4pi] (+360° used to get only positive angles)
        double new_yaw = 360 + new_pose.yaw()*180/M_PI; 
        
        vehiclePoses[i].vehicle_id = new_id;
        vehiclePoses[i].pose.x = new_pose.x();
        vehiclePoses[i].pose.y = new_pose.y();
        vehiclePoses[i].pose.yaw = new_yaw;
    }
}

// Find index of ego vehicle in vehicle_ids
int find_veh_index(vector<uint8_t> &vehicle_ids, int ego_vehicle_id)
{
    vector<uint8_t>::iterator it = find(vehicle_ids.begin(), vehicle_ids.end(), ego_vehicle_id);
    int index = distance(vehicle_ids.begin(), it);

    return index;
}