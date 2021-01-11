//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sort.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jan-2021 13:31:05
//
#ifndef SORT_H
#define SORT_H

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
  extern void b_sort(coder::array<double, 1U> &x, coder::array<int, 1U> &idx);
  extern void sort(int x_data[], const int x_size[1], int idx_data[], int
                   idx_size[1]);
}

#endif

//
// File trailer for sort.h
//
// [EOF]
//
