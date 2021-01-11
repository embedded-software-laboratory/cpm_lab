//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jan-2021 13:31:05
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

// Include Files
#include "main.h"
#include "planTrajectory.h"
#include "planTrajectory_terminate.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Declarations
static void argInit_1x20_real_T(double result[20]);
static void argInit_1x20_struct0_T(mgen::struct0_T result[20]);
static mgen::Pose2D argInit_Pose2D();
static double argInit_real_T();
static mgen::struct0_T argInit_struct0_T();
static unsigned char argInit_uint8_T();
static void main_planTrajectory();

// Function Definitions

//
// Arguments    : double b_x
//                double b_y
//                double b_yaw
// Return Type  : void
//
void Pose2D::init(double b_x, double b_y, double b_yaw)
{
  this->x = b_x;
  this->y = b_y;
  this->yaw = b_yaw;
}

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
  result.init(result_tmp, result_tmp, result_tmp);
  return result;
}

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
  double egoVehicleId_tmp;
  double dv[20];
  mgen::struct0_T rv[20];
  mgen::Pose2D r;
  coder::array<mgen::struct1_T, 1U> trajectory_points;
  boolean_T isPathValid;

  // Initialize function 'planTrajectory' input arguments.
  // Initialize function input argument 'vehicleIdList'.
  // Initialize function input argument 'vehiclePoses'.
  // Initialize function input argument 'goalPose'.
  egoVehicleId_tmp = argInit_real_T();

  // Call the entry-point 'planTrajectory'.
  argInit_1x20_real_T(dv);
  argInit_1x20_struct0_T(rv);
  r = argInit_Pose2D();
  mgen::planTrajectory(dv, rv, &r, egoVehicleId_tmp, egoVehicleId_tmp,
                       trajectory_points, &isPathValid);
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  // The initialize function is being called automatically from your entry-point function. So, a call to initialize is not included here. 
  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_planTrajectory();

  // Terminate the application.
  // You do not need to do this more than one time.
  mgen::planTrajectory_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
