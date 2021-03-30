//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: cosd.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Mar-2021 12:18:40
//

// Include Files
#include "cosd.h"
#include "planTrajectory.h"
#include "planTrajectory_rtwutil.h"
#include "rt_nonfinite.h"
#include "sind.h"
#include <cmath>
#include <string.h>

// Function Definitions

//
// Arguments    : double *x
// Return Type  : void
//
namespace mgen
{
  void b_cosd(double *x)
  {
    if (rtIsInf(*x) || rtIsNaN(*x)) {
      *x = rtNaN;
    } else {
      double absx;
      signed char n;
      *x = rt_remd_snf(*x, 360.0);
      absx = std::abs(*x);
      if (absx > 180.0) {
        if (*x > 0.0) {
          *x -= 360.0;
        } else {
          *x += 360.0;
        }

        absx = std::abs(*x);
      }

      if (absx <= 45.0) {
        *x *= 0.017453292519943295;
        n = 0;
      } else if (absx <= 135.0) {
        if (*x > 0.0) {
          *x = 0.017453292519943295 * (*x - 90.0);
          n = 1;
        } else {
          *x = 0.017453292519943295 * (*x + 90.0);
          n = -1;
        }
      } else if (*x > 0.0) {
        *x = 0.017453292519943295 * (*x - 180.0);
        n = 2;
      } else {
        *x = 0.017453292519943295 * (*x + 180.0);
        n = -2;
      }

      if (n == 0) {
        *x = std::cos(*x);
      } else if (n == 1) {
        *x = -std::sin(*x);
      } else if (n == -1) {
        *x = std::sin(*x);
      } else {
        *x = -std::cos(*x);
      }
    }
  }
}

//
// File trailer for cosd.cpp
//
// [EOF]
//
