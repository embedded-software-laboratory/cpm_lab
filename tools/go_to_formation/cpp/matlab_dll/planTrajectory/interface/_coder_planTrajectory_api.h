/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_planTrajectory_api.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 08-Jan-2021 13:31:05
 */

#ifndef _CODER_PLANTRAJECTORY_API_H
#define _CODER_PLANTRAJECTORY_API_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef struct_Pose2D_tag
#define struct_Pose2D_tag

struct Pose2D_tag
{
  real_T x;
  real_T y;
  real_T yaw;
};

#endif                                 /*struct_Pose2D_tag*/

#ifndef typedef_Pose2D
#define typedef_Pose2D

typedef struct Pose2D_tag Pose2D;

#endif                                 /*typedef_Pose2D*/

#ifndef typedef_struct1_T
#define typedef_struct1_T

typedef struct {
  uint64_T t;
  real_T px;
  real_T py;
  real_T vx;
  real_T vy;
} struct1_T;

#endif                                 /*typedef_struct1_T*/

#ifndef typedef_emxArray_struct1_T
#define typedef_emxArray_struct1_T

typedef struct {
  struct1_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
} emxArray_struct1_T;

#endif                                 /*typedef_emxArray_struct1_T*/

#ifndef typedef_struct0_T
#define typedef_struct0_T

typedef struct {
  uint8_T vehicle_id;
  Pose2D pose;
} struct0_T;

#endif                                 /*typedef_struct0_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#define MAX_THREADS                    omp_get_max_threads()

/* Function Declarations */
extern void planTrajectory(real_T vehicleIdList[20], struct0_T vehiclePoses[20],
  Pose2D *goalPose, real_T egoVehicleId, real_T speed, emxArray_struct1_T
  *trajectory_points, boolean_T *isPathValid);
extern void planTrajectory_api(const mxArray * const prhs[5], int32_T nlhs,
  const mxArray *plhs[2]);
extern void planTrajectory_atexit(void);
extern void planTrajectory_initialize(void);
extern void planTrajectory_terminate(void);
extern void planTrajectory_xil_shutdown(void);
extern void planTrajectory_xil_terminate(void);

#endif

/*
 * File trailer for _coder_planTrajectory_api.h
 *
 * [EOF]
 */
