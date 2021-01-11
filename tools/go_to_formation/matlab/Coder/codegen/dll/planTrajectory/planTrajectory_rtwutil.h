//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: planTrajectory_rtwutil.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jan-2021 13:31:05
//
#ifndef PLANTRAJECTORY_RTWUTIL_H
#define PLANTRAJECTORY_RTWUTIL_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "omp.h"
#include "planTrajectory_types.h"
#define MAX_THREADS                    omp_get_max_threads()

// Function Declarations
namespace mgen
{
  PLANTRAJECTORY_DLL_EXPORT extern int div_s32_floor(int numerator, int
    denominator);
  PLANTRAJECTORY_DLL_EXPORT extern double rt_remd_snf(double u0, double u1);
  PLANTRAJECTORY_DLL_EXPORT extern double rt_roundd_snf(double u);
}

#endif

//
// File trailer for planTrajectory_rtwutil.h
//
// [EOF]
//
