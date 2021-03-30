//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: imdilate.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Mar-2021 12:18:40
//

// Include Files
#include "imdilate.h"
#include "libmwmorphop_binary_tbb.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions

//
// Arguments    : const boolean_T A[180000]
//                boolean_T B[180000]
// Return Type  : void
//
namespace mgen
{
  void imdilate(const boolean_T A[180000], boolean_T B[180000])
  {
    double asizeT[2];
    double nsizeT[2];
    static const boolean_T nhood[289] = { false, false, false, false, false,
      true, true, true, true, true, true, true, false, false, false, false,
      false, false, false, false, true, true, true, true, true, true, true, true,
      true, true, true, false, false, false, false, false, true, true, true,
      true, true, true, true, true, true, true, true, true, true, false, false,
      false, true, true, true, true, true, true, true, true, true, true, true,
      true, true, true, true, false, false, true, true, true, true, true, true,
      true, true, true, true, true, true, true, true, true, false, true, true,
      true, true, true, true, true, true, true, true, true, true, true, true,
      true, true, true, true, true, true, true, true, true, true, true, true,
      true, true, true, true, true, true, true, true, true, true, true, true,
      true, true, true, true, true, true, true, true, true, true, true, true,
      true, true, true, true, true, true, true, true, true, true, true, true,
      true, true, true, true, true, true, true, true, true, true, true, true,
      true, true, true, true, true, true, true, true, true, true, true, true,
      true, true, true, true, true, true, true, true, true, true, true, true,
      true, true, true, true, true, true, true, true, true, true, true, true,
      true, true, true, true, true, true, true, true, true, false, true, true,
      true, true, true, true, true, true, true, true, true, true, true, true,
      true, false, false, true, true, true, true, true, true, true, true, true,
      true, true, true, true, true, true, false, false, false, true, true, true,
      true, true, true, true, true, true, true, true, true, true, false, false,
      false, false, false, true, true, true, true, true, true, true, true, true,
      true, true, false, false, false, false, false, false, false, false, true,
      true, true, true, true, true, true, false, false, false, false, false };

    asizeT[0] = 400.0;
    nsizeT[0] = 17.0;
    asizeT[1] = 450.0;
    nsizeT[1] = 17.0;
    dilate_binary_twod_tbb(A, asizeT, 2.0, nhood, nsizeT, 2.0, B);
  }
}

//
// File trailer for imdilate.cpp
//
// [EOF]
//
