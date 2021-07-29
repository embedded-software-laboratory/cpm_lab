//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pathToTrajectory.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 31-Mar-2021 23:01:38
//
#ifndef PATHTOTRAJECTORY_H
#define PATHTOTRAJECTORY_H

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
  extern void pathToTrajectory(const driving_Path *refPath, double speed, coder::
    array<struct1_T, 1U> &trajectory_points);
}

#endif

//
// File trailer for pathToTrajectory.h
//
// [EOF]
//
