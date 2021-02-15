//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: planTrajectory_data.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 15-Feb-2021 16:41:56
//

// Include Files
#include "planTrajectory_data.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <string.h>

// Variable Definitions
namespace mgen
{
  unsigned int state[625];
  omp_nest_lock_t emlrtNestLockGlobal;
  boolean_T isInitialized_planTrajectory = false;
}

//
// File trailer for planTrajectory_data.cpp
//
// [EOF]
//
