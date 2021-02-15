//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: extremeKElements.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 15-Feb-2021 16:41:56
//
#ifndef EXTREMEKELEMENTS_H
#define EXTREMEKELEMENTS_H

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
  extern void exkib(const coder::array<double, 1U> &a, int k, int idx_data[],
                    int idx_size[1], coder::array<double, 1U> &b);
}

#endif

//
// File trailer for extremeKElements.h
//
// [EOF]
//
