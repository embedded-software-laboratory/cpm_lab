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
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "omp.h"
#include "planTrajectory.h"
#include "planTrajectory_terminate.h"
#include "planTrajectory_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string>

#define MAX_THREADS                    omp_get_max_threads()
#define _USE_MATH_DEFINES


#include "cpm/Logging.hpp"                      //->cpm_lib->include->cpm
#include "cpm/CommandLineReader.hpp"            //->cpm_lib->include->cpm
#include "cpm/init.hpp"                         //->cpm_lib->include->cpm
#include "cpm/MultiVehicleReader.hpp"           //->cpm_lib->include->cpm
#include "cpm/ParticipantSingleton.hpp"         //->cpm_lib->include->cpm
#include "cpm/Timer.hpp"                        //->cpm_lib->include->cpm
#include "cpm/Writer.hpp"
#include "VehicleObservation.hpp" 
#include "VehicleCommandTrajectory.hpp"
#include "Pose2D.hpp"
#include <iostream>
#include <sstream>
#include <memory>
#include <mutex>
#include <thread>
#include <map>
#include <cmath>
#include <chrono>
#include <ctime>

using std::vector;


// Function Declarations
static void argInit_1xd256_real_T(double result_data[], int result_size[2]);
static void argInit_1xd256_struct0_T(mgen::struct0_T result_data[], int
  result_size[2]);
static mgen::Pose2D argInit_Pose2D();
static double argInit_real_T();
static mgen::struct0_T argInit_struct0_T();
static unsigned char argInit_uint8_T();
static void main_planTrajectory();

void set_home_poses(int n_vehicles,  vector<mgen::Pose2D> &home_poses);

// Function Definitions

//
// Arguments    : double result_data[]
//                int result_size[2]
// Return Type  : void
//
static void argInit_1xd256_real_T(double result_data[], int result_size[2])
{
  // Set the size of the array.
  // Change this size to the value that the application requires.
  result_size[0] = 1;
  result_size[1] = 2;

  // Loop over the array to initialize each element.
  for (int idx1 = 0; idx1 < 2; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result_data[idx1] = argInit_real_T();
  }
}

//
// Arguments    : mgen::struct0_T result_data[]
//                int result_size[2]
// Return Type  : void
//
static void argInit_1xd256_struct0_T(mgen::struct0_T result_data[], int
  result_size[2])
{
  // Set the size of the array.
  // Change this size to the value that the application requires.
  result_size[0] = 1;
  result_size[1] = 2;

  // Loop over the array to initialize each element.
  for (int idx1 = 0; idx1 < 2; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result_data[idx1] = argInit_struct0_T();
  }
}

//
// Arguments    : void
// Return Type  : mgen::Pose2D
//
static mgen::Pose2D argInit_Pose2D()
{
  mgen::Pose2D result;
  double result_tmp;

  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result_tmp = argInit_real_T();
  result.x = result_tmp;
  result.y = result_tmp;
  result.yaw = result_tmp;
  //result.init(result_tmp, result_tmp, result_tmp);
  return result;
}

//
// Arguments    : double b_x
//                double b_y
//                double b_yaw
// Return Type  : void

/*
void Pose2D::init(double b_x, double b_y, double b_yaw)
{
  this->x = b_x;
  this->y = b_y;
  this->yaw = b_yaw;
}*/

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : mgen::struct0_T
//
static mgen::struct0_T argInit_struct0_T()
{
  mgen::struct0_T result;

  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result.vehicle_id = argInit_uint8_T();
  result.pose = argInit_Pose2D();
  return result;
}

//
// Arguments    : void
// Return Type  : unsigned char
//
static unsigned char argInit_uint8_T()
{
  return 0U;
}

void set_home_poses(int n_vehicles,  vector<mgen::Pose2D> &home_poses)
{ 
  mgen::Pose2D home_pose = argInit_Pose2D();
  double map_y = 4.0;

  // vehicle dimensions [m]
  double  vehicle_rear_overhang = 0.03;
  double  vehicle_length = 0.2200;
  double  vehicle_width = 0.1070;
  double  clearance = vehicle_width/2; //safety distance vehicle to vehicle and vehicle to map edge [m]

  // arrangement of vehicles
  int max_no_vehicles = 20;
  int total_rows = 3;
  int total_columns = ceil(max_no_vehicles / total_rows);
  
  int column;
  int row;

  for (int i = 1; i < n_vehicles+1; i++) {
      
      if (i % total_columns == 0){
        column = total_columns; 
        row = floor(i / total_columns);
      }else{
        column = i % total_columns;
        row = 1 + floor(i / total_columns);
      }

      home_pose.x = 2 * vehicle_width + column * clearance + (column-1) * vehicle_width ;
      home_pose.y = map_y - (vehicle_length - vehicle_rear_overhang) - row * clearance - (row-1) * vehicle_length * 2 ;
      home_pose.yaw = 90;
      home_poses.push_back(home_pose);
    }
}

int main(int argc, char *argv[])
{

  //Initialize cpm library
  const std::string node_id = "mlib_test";
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

  bool computed = false;
  bool planning_started = false;
  bool trajectory_is_active = false;
  double trajectory_duration;
  double reference_time;
  uint64_t invalid_after_stamp;
  vector<TrajectoryPoint> trajectory_points;
  vector<std::pair<int, Pose2D>> help_vector;
  int vehicle_id = 1;
  boolean_T is_path_valid;

  // setting of home poses
  vector<mgen::Pose2D> goal_poses;
  set_home_poses(vehicle_ids.size(), goal_poses);
  
  auto timer = cpm::Timer::create("mlib_test", dt_nanos, 0, false, true, enable_simulated_time); 
  timer->start([&](uint64_t t_now)
  {
  
    if(! planning_started){
      reference_time = t_now;
      planning_started = true;
    }

    if(! computed){
          
          std::map<uint8_t, VehicleObservation> ips_sample;
          std::map<uint8_t, uint64_t> ips_sample_age;
          ips_reader.get_samples(t_now, ips_sample, ips_sample_age);
          
          int new_id;
          Pose2D new_pose;

          // Init variables needed as arguments by matlab generated function
          double vehicleIdList_data[256];
          int vehicleIdList_size[2];
          mgen::struct0_T vehiclePoses_data[256];
          int vehiclePoses_size[2];
          mgen::Pose2D r;
          coder::array<mgen::struct1_T, 1U> generated_trajectory;
          

          // Initialize function 'planTrajectory' input arguments.
          // Initialize function input argument 'vehicleIdList'.
          argInit_1xd256_real_T(vehicleIdList_data, vehicleIdList_size);

          // Initialize function input argument 'vehiclePoses'.
          argInit_1xd256_struct0_T(vehiclePoses_data, vehiclePoses_size);

          // Initialize function input argument 'goalPose'.
          // Call the entry-point 'planTrajectory'.
          r = argInit_Pose2D();

          for (int i = 0; i < 20; i++) {
            vehicleIdList_data[i] = i+1;
          }
          vehicleIdList_size[1] = 20;

          
          for(auto i = ips_sample.begin(); i != ips_sample.end(); i++){
            auto data = i->second;
            //new_id = data.vehicle_id();
            //new_pose = data.pose();
            help_vector.emplace_back(data.vehicle_id(), data.pose());
          }

          for(int i = 0; i < vehicle_ids.size(); i++){
            new_pose = help_vector[i].second;
            new_id = help_vector[i].first;

            // conversion from rad[-pi,pi] to deg [0, 4pi] (+360° used to get only positive angles)
            double new_yaw = 360 + new_pose.yaw()*180/M_PI; 
            vehiclePoses_data[i].vehicle_id = new_id;
            vehiclePoses_data[i].pose.x = new_pose.x();
            vehiclePoses_data[i].pose.y = new_pose.y();
            vehiclePoses_data[i].pose.yaw = new_yaw;
          }
          
        vehiclePoses_size[1] = 1;
        double speed = 1.0;

        std::cout << "x" << goal_poses[0].x << "y" << goal_poses[0].y << "yaw" << goal_poses[0].yaw << std::endl;
              
        mgen::planTrajectory(vehicleIdList_data, vehicleIdList_size, vehiclePoses_data,
                            vehiclePoses_size, &goal_poses[0], vehicle_id, speed,
                            generated_trajectory, &is_path_valid);
        
        auto len = *(generated_trajectory).size();
        
        for (int i = 0; i < len; i++) {
          std::cout << " t: " << generated_trajectory[i].t << " px: " << generated_trajectory[i].px << " py: " << generated_trajectory[i].py << " vx: " << 
                    generated_trajectory[i].vx << " vy: " << generated_trajectory[i].vy << std::endl;
        }
        std::cout << "path validity: " << (is_path_valid == 1) << std::endl;
        
        if(is_path_valid){
          
          for (size_t i = 0; i < len; ++i)
          {
            TrajectoryPoint trajectory_point;
            trajectory_point.px(generated_trajectory[i].px);
            trajectory_point.py(generated_trajectory[i].py);
            trajectory_point.vx(generated_trajectory[i].vx);
            trajectory_point.vy(generated_trajectory[i].vy);
            trajectory_point.t().nanoseconds(generated_trajectory[i].t + t_now);
            trajectory_points.push_back(trajectory_point);

          } 

          invalid_after_stamp = trajectory_points.back().t().nanoseconds() + 2000000000ull;
          std::cout << "invalid after: " << invalid_after_stamp << std::endl;
          computed = true;
        }
  }

    if(is_path_valid && computed && t_now < invalid_after_stamp){
            std::cout << "sending trajectory" << std::endl;
            // Send the current trajectory
            rti::core::vector<TrajectoryPoint> rti_trajectory_points(trajectory_points);
            VehicleCommandTrajectory vehicle_command_trajectory;
            //vehicle_command_trajectory.vehicle_id(vehicle_ids.at(0));
            vehicle_command_trajectory.vehicle_id(vehicle_id);
            vehicle_command_trajectory.trajectory_points(rti_trajectory_points);
            vehicle_command_trajectory.header().create_stamp().nanoseconds(t_now);
            vehicle_command_trajectory.header().valid_after_stamp().nanoseconds(t_now + 1000000000ull);
            writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);

    } else {
      std::cout << "invalid after stamp: " << invalid_after_stamp << std::endl;
      std::cout << "t_now: " << t_now << std::endl;
      std::cout << "stopping timer" << std::endl;
      timer->stop();
    }


  });

  // Terminate the application.
  // You do not need to do this more than one time.
  mgen::planTrajectory_terminate();
  return 0;
}