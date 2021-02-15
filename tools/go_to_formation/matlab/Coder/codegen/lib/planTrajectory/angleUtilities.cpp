//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: angleUtilities.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 15-Feb-2021 16:41:56
//

// Include Files
#include "angleUtilities.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <string.h>

// Function Definitions

//
// Arguments    : double *theta
// Return Type  : void
//
namespace mgen
{
  void angleUtilities::wrapToPi(double *theta)
  {
    double d;
    d = *theta + 3.1415926535897931;
    angleUtilities::wrapTo2Pi((&d));
    *theta = d - 3.1415926535897931;
  }

  //
  // Arguments    : double x
  //                double y
  // Return Type  : double
  //
  double angleUtilities::angdiff(double x, double y)
  {
    double delta;
    delta = y - x;
    angleUtilities::wrapToPi((&delta));
    return delta;
  }

  //
  // Arguments    : double *theta
  // Return Type  : void
  //
  void angleUtilities::convertAndWrapTo2Pi(double *theta)
  {
    *theta *= 0.017453292519943295;
    angleUtilities::wrapTo2Pi((theta));
  }

  //
  // Arguments    : double *theta
  // Return Type  : void
  //
  void angleUtilities::wrapTo2Pi(double *theta)
  {
    boolean_T positiveInput;
    double x;
    positiveInput = (*theta > 0.0);
    x = *theta;
    if (rtIsNaN(x) || rtIsInf(x)) {
      *theta = rtNaN;
    } else if (x == 0.0) {
      *theta = 0.0;
    } else {
      boolean_T rEQ0;
      *theta = std::fmod(x, 6.2831853071795862);
      rEQ0 = (*theta == 0.0);
      if (!rEQ0) {
        double q;
        q = std::abs(x / 6.2831853071795862);
        rEQ0 = !(std::abs(q - std::floor(q + 0.5)) > 2.2204460492503131E-16 * q);
      }

      if (rEQ0) {
        *theta = 0.0;
      } else {
        if (x < 0.0) {
          *theta += 6.2831853071795862;
        }
      }
    }

    *theta += 6.2831853071795862 * static_cast<double>((*theta == 0.0) &&
      positiveInput);
  }
}

//
// File trailer for angleUtilities.cpp
//
// [EOF]
//
