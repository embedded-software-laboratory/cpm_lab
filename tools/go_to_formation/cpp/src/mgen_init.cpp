#include "../include/mgen_init.hpp"

#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "omp.h"
#include "planTrajectory_types.h"
#define MAX_THREADS                    omp_get_max_threads()


//
// Arguments    : double result_data[]
//                int result_size[2]
// Return Type  : void
//
void argInit_1xd256_real_T(double result_data[], int result_size[2])
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
void argInit_1xd256_struct0_T(mgen::struct0_T result_data[], int
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
mgen::Pose2D argInit_Pose2D()
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
// Arguments    : void
// Return Type  : double
//
double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : mgen::struct0_T
//
mgen::struct0_T argInit_struct0_T()
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
unsigned char argInit_uint8_T()
{
  return 0U;
}