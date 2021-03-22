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

using std::vector;

//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Feb-2021 11:34:31
//

//***********************************************************************
// This automatically generated example C++ main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************

// Function Declarations
static void argInit_1xd256_real_T(double result_data[], int result_size[2]);
static void argInit_1xd256_struct0_T(mgen::struct0_T result_data[], int
  result_size[2]);
static mgen::Pose2D argInit_Pose2D();
static double argInit_real_T();
static mgen::struct0_T argInit_struct0_T();
static unsigned char argInit_uint8_T();
static void main_planTrajectory();

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

//
// Arguments    : void
// Return Type  : void
//

static void main_planTrajectory()
{
  double vehicleIdList_data[256];
  int vehicleIdList_size[2];
  mgen::struct0_T vehiclePoses_data[256];
  int vehiclePoses_size[2];
  mgen::Pose2D r;
  coder::array<mgen::struct1_T, 1U> generated_trajectory;
  boolean_T isPathValid;

  // Initialize function 'planTrajectory' input arguments.
  // Initialize function input argument 'vehicleIdList'.
  argInit_1xd256_real_T(vehicleIdList_data, vehicleIdList_size);

  // Initialize function input argument 'vehiclePoses'.
  argInit_1xd256_struct0_T(vehiclePoses_data, vehiclePoses_size);

  // Initialize function input argument 'goalPose'.
  r = argInit_Pose2D();

  for (int i = 0; i < 20; i++) {
    vehicleIdList_data[i] = i+1;
  }
  vehicleIdList_size[1] = 20;

  vehiclePoses_data[0].vehicle_id = 3;
  //vehiclePoses_data[0].pose.x = 4.0;
  //vehiclePoses_data[0].pose.y = 3.5;
  //vehiclePoses_data[0].pose.yaw = 180;

  vehiclePoses_data[0].pose.x = 3.87879584432267;
  vehiclePoses_data[0].pose.y = 3.72046857262141;
  vehiclePoses_data[0].pose.yaw = 330; 

  vehiclePoses_size[1] = 1;

  r.x = 0.2;
  r.y = 3.6;
  r.yaw = 90; //deg - value received will have to be converted from rad

  double speed = 1.0;
  int vehicle_id = 3;
 
  mgen::planTrajectory(vehicleIdList_data, vehicleIdList_size, vehiclePoses_data,
                       vehiclePoses_size, &r, vehicle_id, speed,
                       generated_trajectory, &isPathValid);
  
  auto len = *(generated_trajectory).size();
  
  for (int i = 0; i < len; i++) {
    std::cout << " t: " << generated_trajectory[i].t << " px: " << generated_trajectory[i].px << " py: " << generated_trajectory[i].py << " vx: " << 
              generated_trajectory[i].vx << " vy: " << generated_trajectory[i].vy << std::endl;
  }
  std::cout << "path validity: " << (isPathValid == 1) << std::endl;

}


int main(int argc, char *argv[])
{

  main_planTrajectory();

  // Terminate the application.
  // You do not need to do this more than one time.
  mgen::planTrajectory_terminate();
  return 0;
}