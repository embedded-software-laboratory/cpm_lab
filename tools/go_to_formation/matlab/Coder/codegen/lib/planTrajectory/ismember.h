//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ismember.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Mar-2021 12:18:40
//
#ifndef ISMEMBER_H
#define ISMEMBER_H

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
  extern void local_ismember(const coder::array<double, 2U> &a, const coder::
    array<double, 2U> &s, coder::array<boolean_T, 2U> &tf);
}

#endif

//
// File trailer for ismember.h
//
// [EOF]
//
