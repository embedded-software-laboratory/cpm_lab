//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sortIdx.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jan-2021 13:31:05
//
#ifndef SORTIDX_H
#define SORTIDX_H

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
  extern void b_sortIdx(const coder::array<double, 2U> &x, coder::array<int, 2U>
                        &idx);
  extern void c_sortIdx(int x_data[], const int x_size[1], int idx_data[], int
                        idx_size[1]);
  extern void d_sortIdx(coder::array<double, 1U> &x, coder::array<int, 1U> &idx);
  extern void sortIdx(const coder::array<double, 1U> &x, int idx_data[], int
                      idx_size[1]);
}

#endif

//
// File trailer for sortIdx.h
//
// [EOF]
//
