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

// Include Files
#include <cstddef>
#include <cstdlib>
#include "matlab_dll/planTrajectory/rtwtypes.h"
#include "matlab_dll/planTrajectory/planTrajectory_types.h"
#include "matlab_dll/planTrajectory/planTrajectory.h"
#include "matlab_dll/planTrajectory/planTrajectory_terminate.h"
#include "matlab_dll/planTrajectory/rt_nonfinite.h"
#include <string.h>

#include "cpm/Logging.hpp"                      //->cpm_lib->include->cpm
#include "cpm/CommandLineReader.hpp"            //->cpm_lib->include->cpm
#include "cpm/init.hpp"                         //->cpm_lib->include->cpm
#include "cpm/MultiVehicleReader.hpp"           //->cpm_lib->include->cpm
#include "cpm/ParticipantSingleton.hpp"         //->cpm_lib->include->cpm
#include "cpm/Timer.hpp"                        //->cpm_lib->include->cpm
#include "cpm/Writer.hpp"
#include "VehicleObservation.hpp" 
#include "VehicleCommandTrajectory.hpp"
#include <iostream>
#include <sstream>
#include <memory>
#include <mutex>
#include <thread>
#include <map>

using std::vector;

// Matlab lib function Declarations
static void argInit_1x20_real_T(double result[20]);
static void argInit_1x20_struct0_T(mgen::struct0_T result[20]);
static mgen::Pose2D argInit_Pose2D();
static double argInit_real_T();
static mgen::struct0_T argInit_struct0_T();
static unsigned char argInit_uint8_T();

// Matlab lib function Definitions

// Arguments    : double b_x
//                double b_y
//                double b_yaw
// Return Type  : void
//
/*
void Pose2D::init(double b_x, double b_y, double b_yaw)
{
  this->x = b_x;
  this->y = b_y;
  this->yaw = b_yaw;
}*/

//
// Arguments    : double result[20]
// Return Type  : void
//
static void argInit_1x20_real_T(double result[20])
{
  // Loop over the array to initialize each element.
  for (int idx1 = 0; idx1 < 20; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx1] = argInit_real_T();
  }
}

//
// Arguments    : mgen::struct0_T result[20]
// Return Type  : void
//
static void argInit_1x20_struct0_T(mgen::struct0_T result[20])
{
  // Loop over the array to initialize each element.
  for (int idx1 = 0; idx1 < 20; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx1] = argInit_struct0_T();
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
  //result.init(result_tmp, result_tmp, result_tmp);
  result.x = result_tmp;
  result.y = result_tmp;
  result.yaw = result_tmp;
  return result;
}

//
// Arguments    : voids
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



int main(int argc, char *argv[])
{   //////////////////Set logging details///////////////////////////////////////////////////////////
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id("reader_test");
    const bool enable_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv); //variable is set to false 
    ////////////////Set vehicle IDs for the vehicles selected in the command line or the LCC////////
    const std::vector<int> vehicle_ids_int = cpm::cmd_parameter_ints("vehicle_ids", {1}, argc, argv);
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
   
    ///////////// writer and reader for sending trajectory commands////////////////////////
    //the writer will write data for the trajectory for the position of the vehicle (x,y) and the speed for each direction vecotr (vx,vy) and the vehicle ID
    cpm::Writer<VehicleCommandTrajectory> writer_vehicleCommandTrajectory("vehicleCommandTrajectory");
    //the reader will read the pose of a vehicle given by its vehicle ID
    cpm::MultiVehicleReader<VehicleObservation> ips_reader(
        cpm::get_topic<VehicleObservation>("vehicleObservation"),
        vehicle_ids
    );

    int count = 0;
    
    //create(node_id, period in nanoseconds, offset in nanoseconds, bool wait_for_start, bool simulated_time_allowed, bool simulated_time (set in line 27))
    auto timer = cpm::Timer::create("reader_test", dt_nanos, 0, false, true, enable_simulated_time); 
    timer->start([&](uint64_t t_now)
    {
        std::map<uint8_t, VehicleObservation> ips_sample;
        std::map<uint8_t, uint64_t> ips_sample_age;
        ips_reader.get_samples(t_now, ips_sample, ips_sample_age);     
        
        mgen::struct0_T vehiclePoses[20];
        argInit_1x20_struct0_T(vehiclePoses);

        double new_id = 0;
        double x = 0;
        double y = 0;
        double yaw = 0;
        

        for(auto e:ips_sample)
        {
            auto data = e.second;
            auto new_pose = data.pose();
            new_id = data.vehicle_id();
            x = double(new_pose.x());
            y = double(new_pose.y());
            yaw = double(new_pose.yaw());
            
  
            std::cout << new_id << std::endl;
            std::cout << new_pose << std::endl;

          
            count += 1;
            std::cout << count << std::endl;

        }

        vehiclePoses[0].vehicle_id = new_id;
        vehiclePoses[0].pose.x = x;
        vehiclePoses[0].pose.y = y;
        vehiclePoses[0].pose.yaw = yaw;

        double ego_vehicle_id = new_id;
        double speed = 1; // [m/s]
        coder::array<mgen::struct1_T, 1U> trajectory_points;
        boolean_T isPathValid;

        // Initialize function 'planTrajectory' input arguments.
        // Initialize function input argument 'vehicleIdList'.
        // Initialize function input argument 'vehiclePoses'.
        // Initialize function input argument 'goalPose'.
         mgen::Pose2D goalPose = argInit_Pose2D();
         goalPose.x = 2.0;
         goalPose.y = 2.0;
         goalPose.yaw = 0.0;

        mgen::planTrajectory(vehiclePoses, &goalPose, ego_vehicle_id, speed,
                            trajectory_points, &isPathValid);
        

    
        /*
        if ((count > 10)|| (new_id != 0) )
        {
        timer->stop();
        } */
    });

    // Terminate the application.
    // You do not need to do this more than one time.
    mgen::planTrajectory_terminate();
    return 0;

}