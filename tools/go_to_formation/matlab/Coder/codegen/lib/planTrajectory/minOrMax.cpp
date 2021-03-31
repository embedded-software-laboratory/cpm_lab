//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: minOrMax.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 31-Mar-2021 23:01:38
//

// Include Files
#include "minOrMax.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions

//
// Arguments    : const double x_data[]
//                const int x_size[2]
//                double ex_data[]
//                int ex_size[2]
// Return Type  : void
//
namespace mgen
{
  void maximum2(const double x_data[], const int x_size[2], double ex_data[],
                int ex_size[2])
  {
    signed char unnamed_idx_0;
    signed char unnamed_idx_1;
    unnamed_idx_0 = static_cast<signed char>(x_size[0]);
    unnamed_idx_1 = static_cast<signed char>(x_size[1]);
    ex_size[0] = unnamed_idx_0;
    ex_size[1] = unnamed_idx_1;
    if (0 <= unnamed_idx_0 * unnamed_idx_1 - 1) {
      if (x_data[0] > 1.0) {
        ex_data[0] = x_data[0];
      } else {
        ex_data[0] = 1.0;
      }
    }
  }
}

//
// File trailer for minOrMax.cpp
//
// [EOF]
//
