//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: planTrajectory_rtwutil.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 31-Mar-2021 23:01:38
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
  extern int div_s32_floor(int numerator, int denominator);
  extern double rt_remd_snf(double u0, double u1);
  extern double rt_roundd_snf(double u);
}

#endif

//
// File trailer for planTrajectory_rtwutil.h
//
// [EOF]
//
