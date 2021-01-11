//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: planTrajectory_terminate.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jan-2021 13:31:05
//
#ifndef PLANTRAJECTORY_TERMINATE_H
#define PLANTRAJECTORY_TERMINATE_H

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
  PLANTRAJECTORY_DLL_EXPORT extern void planTrajectory_terminate();
}

#endif

//
// File trailer for planTrajectory_terminate.h
//
// [EOF]
//
