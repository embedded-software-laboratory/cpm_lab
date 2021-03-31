//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: planTrajectory_terminate.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 31-Mar-2021 23:01:38
//

// Include Files
#include "planTrajectory_terminate.h"
#include "planTrajectory.h"
#include "planTrajectory_data.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
namespace mgen
{
  void planTrajectory_terminate()
  {
    omp_destroy_nest_lock(&emlrtNestLockGlobal);
    isInitialized_planTrajectory = false;
  }
}

//
// File trailer for planTrajectory_terminate.cpp
//
// [EOF]
//
