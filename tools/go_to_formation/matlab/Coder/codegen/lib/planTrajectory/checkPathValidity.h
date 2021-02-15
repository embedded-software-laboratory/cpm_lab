//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: checkPathValidity.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 15-Feb-2021 16:41:56
//
#ifndef CHECKPATHVALIDITY_H
#define CHECKPATHVALIDITY_H

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
  extern void computeSamples(const driving_Path *path, coder::array<double, 2U>
    &pathSamples);
}

#endif

//
// File trailer for checkPathValidity.h
//
// [EOF]
//
