//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eml_setop.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jan-2021 13:31:05
//
#ifndef EML_SETOP_H
#define EML_SETOP_H

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
  extern void do_vectors(const unsigned int a_data[], const int a_size[2], const
    coder::array<double, 2U> &b, unsigned int c_data[], int c_size[2], int
    ia_data[], int ia_size[1], int ib_data[], int ib_size[1]);
}

#endif

//
// File trailer for eml_setop.h
//
// [EOF]
//
