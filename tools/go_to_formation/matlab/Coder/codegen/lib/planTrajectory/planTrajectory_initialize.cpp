//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: planTrajectory_initialize.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 31-Mar-2021 23:01:38
//

// Include Files
#include "planTrajectory_initialize.h"
#include "eml_rand_mt19937ar_stateful.h"
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
  void planTrajectory_initialize()
  {
    rt_InitInfAndNaN();
    omp_init_nest_lock(&emlrtNestLockGlobal);
    c_eml_rand_mt19937ar_stateful_i();
    isInitialized_planTrajectory = true;
  }
}

//
// File trailer for planTrajectory_initialize.cpp
//
// [EOF]
//
