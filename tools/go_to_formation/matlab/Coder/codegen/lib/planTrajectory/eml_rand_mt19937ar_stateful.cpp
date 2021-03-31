//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eml_rand_mt19937ar_stateful.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 31-Mar-2021 23:01:38
//

// Include Files
#include "eml_rand_mt19937ar_stateful.h"
#include "planTrajectory.h"
#include "planTrajectory_data.h"
#include "rt_nonfinite.h"
#include <cstring>
#include <string.h>

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
namespace mgen
{
  void c_eml_rand_mt19937ar_stateful_i()
  {
    unsigned int r;
    std::memset(&state[0], 0, 625U * sizeof(unsigned int));
    r = 5489U;
    state[0] = 5489U;
    for (int mti = 0; mti < 623; mti++) {
      r = ((r ^ r >> 30U) * 1812433253U + mti) + 1U;
      state[mti + 1] = r;
    }

    state[624] = 624U;
  }
}

//
// File trailer for eml_rand_mt19937ar_stateful.cpp
//
// [EOF]
//
