//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: all.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jan-2021 13:31:05
//

// Include Files
#include "all.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions

//
// Arguments    : const coder::array<boolean_T, 2U> *x
//                coder::array<boolean_T, 1U> *y
// Return Type  : void
//
namespace mgen
{
  void all(const coder::array<boolean_T, 2U> &x, coder::array<boolean_T, 1U> &y)
  {
    unsigned int outsize_idx_0;
    int vstride;
    int i2;
    int iy;
    int i1;
    int ix;
    outsize_idx_0 = static_cast<unsigned int>(x.size(0));
    y.set_size((static_cast<int>(outsize_idx_0)));
    vstride = static_cast<int>(outsize_idx_0);
    for (i2 = 0; i2 < vstride; i2++) {
      y[i2] = true;
    }

    vstride = x.size(0);
    i2 = x.size(0) << 1;
    iy = -1;
    i1 = 0;
    for (int j = 0; j < vstride; j++) {
      boolean_T exitg1;
      i1++;
      i2++;
      iy++;
      ix = i1;
      exitg1 = false;
      while ((!exitg1) && ((vstride > 0) && (ix <= i2))) {
        if (!x[ix - 1]) {
          y[iy] = false;
          exitg1 = true;
        } else {
          ix += vstride;
        }
      }
    }
  }
}

//
// File trailer for all.cpp
//
// [EOF]
//
