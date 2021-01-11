//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: planTrajectory_data.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jan-2021 13:31:05
//
#ifndef PLANTRAJECTORY_DATA_H
#define PLANTRAJECTORY_DATA_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "omp.h"
#include "planTrajectory_types.h"

// Variable Declarations
namespace mgen
{
  extern unsigned int state[625];
  extern omp_nest_lock_t emlrtNestLockGlobal;
  extern boolean_T isInitialized_planTrajectory;
}

#define MAX_THREADS                    omp_get_max_threads()
#endif

//
// File trailer for planTrajectory_data.h
//
// [EOF]
//
