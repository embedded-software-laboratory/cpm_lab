//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: minOrMax.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 31-Mar-2021 23:01:38
//
#ifndef MINORMAX_H
#define MINORMAX_H

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
  extern void maximum2(const double x_data[], const int x_size[2], double
                       ex_data[], int ex_size[2]);
}

#endif

//
// File trailer for minOrMax.h
//
// [EOF]
//
