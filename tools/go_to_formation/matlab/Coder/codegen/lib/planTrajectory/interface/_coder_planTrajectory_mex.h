/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_planTrajectory_mex.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 31-Mar-2021 23:01:38
 */

#ifndef _CODER_PLANTRAJECTORY_MEX_H
#define _CODER_PLANTRAJECTORY_MEX_H

/* Include Files */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include "_coder_planTrajectory_api.h"
#define MAX_THREADS                    omp_get_max_threads()

/* Function Declarations */
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
  const mxArray *prhs[]);
extern emlrtCTX mexFunctionCreateRootTLS(void);

#endif

/*
 * File trailer for _coder_planTrajectory_mex.h
 *
 * [EOF]
 */
