//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: unique.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Mar-2021 12:18:40
//
#ifndef UNIQUE_H
#define UNIQUE_H

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
  extern void b_unique_vector(const coder::array<double, 1U> &a, coder::array<
    double, 1U> &b);
  extern void unique_vector(const coder::array<double, 2U> &a, coder::array<
    double, 2U> &b);
}

#endif

//
// File trailer for unique.h
//
// [EOF]
//
