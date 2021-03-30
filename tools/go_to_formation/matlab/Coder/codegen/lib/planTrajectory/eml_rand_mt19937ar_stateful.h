//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eml_rand_mt19937ar_stateful.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Mar-2021 12:18:40
//
#ifndef EML_RAND_MT19937AR_STATEFUL_H
#define EML_RAND_MT19937AR_STATEFUL_H

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
  extern void c_eml_rand_mt19937ar_stateful_i();
}

#endif

//
// File trailer for eml_rand_mt19937ar_stateful.h
//
// [EOF]
//
