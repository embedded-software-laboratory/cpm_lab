//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: planTrajectory_rtwutil.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Mar-2021 12:18:40
//

// Include Files
#include "planTrajectory_rtwutil.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <cfloat>
#include <cmath>
#include <string.h>

// Function Definitions

//
// Arguments    : int numerator
//                int denominator
// Return Type  : int
//
namespace mgen
{
  int div_s32_floor(int numerator, int denominator)
  {
    int quotient;
    unsigned int absNumerator;
    if (denominator == 0) {
      if (numerator >= 0) {
        quotient = MAX_int32_T;
      } else {
        quotient = MIN_int32_T;
      }
    } else {
      unsigned int absDenominator;
      boolean_T quotientNeedsNegation;
      unsigned int tempAbsQuotient;
      if (numerator < 0) {
        absNumerator = ~static_cast<unsigned int>(numerator) + 1U;
      } else {
        absNumerator = static_cast<unsigned int>(numerator);
      }

      if (denominator < 0) {
        absDenominator = ~static_cast<unsigned int>(denominator) + 1U;
      } else {
        absDenominator = static_cast<unsigned int>(denominator);
      }

      quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
      tempAbsQuotient = absNumerator / absDenominator;
      if (quotientNeedsNegation) {
        absNumerator %= absDenominator;
        if (absNumerator > 0U) {
          tempAbsQuotient++;
        }

        quotient = -static_cast<int>(tempAbsQuotient);
      } else {
        quotient = static_cast<int>(tempAbsQuotient);
      }
    }

    return quotient;
  }

  //
  // Arguments    : double u0
  //                double u1
  // Return Type  : double
  //
  double rt_remd_snf(double u0, double u1)
  {
    double y;
    if (rtIsNaN(u0) || rtIsNaN(u1) || rtIsInf(u0)) {
      y = rtNaN;
    } else if (rtIsInf(u1)) {
      y = u0;
    } else {
      double b_u1;
      if (u1 < 0.0) {
        b_u1 = std::ceil(u1);
      } else {
        b_u1 = std::floor(u1);
      }

      if ((u1 != 0.0) && (u1 != b_u1)) {
        b_u1 = std::abs(u0 / u1);
        if (!(std::abs(b_u1 - std::floor(b_u1 + 0.5)) > DBL_EPSILON * b_u1)) {
          y = 0.0 * u0;
        } else {
          y = std::fmod(u0, u1);
        }
      } else {
        y = std::fmod(u0, u1);
      }
    }

    return y;
  }

  //
  // Arguments    : double u
  // Return Type  : double
  //
  double rt_roundd_snf(double u)
  {
    double y;
    if (std::abs(u) < 4.503599627370496E+15) {
      if (u >= 0.5) {
        y = std::floor(u + 0.5);
      } else if (u > -0.5) {
        y = u * 0.0;
      } else {
        y = std::ceil(u - 0.5);
      }
    } else {
      y = u;
    }

    return y;
  }
}

//
// File trailer for planTrajectory_rtwutil.cpp
//
// [EOF]
//
