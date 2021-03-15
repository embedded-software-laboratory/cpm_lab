#pragma once

#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "omp.h"
#include "planTrajectory_types.h"
#define MAX_THREADS                    omp_get_max_threads()

// Function Declarations
void argInit_1xd256_real_T(double result_data[], int result_size[2]);
void argInit_1xd256_struct0_T(mgen::struct0_T result_data[], int result_size[2]);
mgen::Pose2D argInit_Pose2D();
double argInit_real_T();
mgen::struct0_T argInit_struct0_T();
unsigned char argInit_uint8_T();
