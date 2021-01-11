//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: planTrajectory.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jan-2021 13:31:05
//
#ifndef PLANTRAJECTORY_H
#define PLANTRAJECTORY_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "omp.h"
#include "planTrajectory_types.h"
#define MAX_THREADS                    omp_get_max_threads()

// Function Declarations
namespace mgen
{
  PLANTRAJECTORY_DLL_EXPORT extern void planTrajectory(const double
    vehicleIdList[20], const struct0_T vehiclePoses[20], const Pose2D *goalPose,
    double egoVehicleId, double speed, coder::array<struct1_T, 1U>
    &trajectory_points, boolean_T *isPathValid);
}

#endif

//
// File trailer for planTrajectory.h
//
// [EOF]
//
