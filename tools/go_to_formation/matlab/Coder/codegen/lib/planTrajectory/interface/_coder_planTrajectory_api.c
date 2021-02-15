/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_planTrajectory_api.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 15-Feb-2021 16:41:56
 */

/* Include Files */
#include "_coder_planTrajectory_api.h"
#include "_coder_planTrajectory_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131594U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "planTrajectory",                    /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T **y_data, int32_T y_size[2]);
static const mxArray *b_emlrt_marshallOut(const boolean_T u);
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *vehiclePoses,
  const char_T *identifier, struct0_T y_data[], int32_T y_size[2]);
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct0_T y_data[], int32_T y_size[2]);
static uint8_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *vehicleIdList,
  const char_T *identifier, real_T **y_data, int32_T y_size[2]);
static const mxArray *emlrt_marshallOut(const emxArray_struct1_T *u);
static void emxFree_struct1_T(emxArray_struct1_T **pEmxArray);
static void emxInit_struct1_T(const emlrtStack *sp, emxArray_struct1_T
  **pEmxArray, int32_T numDimensions, boolean_T doPush);
static Pose2D f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static real_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static Pose2D h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *goalPose,
  const char_T *identifier);
static uint8_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *egoVehicleId, const char_T *identifier);
static real_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *speed,
  const char_T *identifier);
static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T **ret_data, int32_T ret_size[2]);
static uint8_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);
static real_T m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);

/* Function Definitions */

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T **y_data
 *                int32_T y_size[2]
 * Return Type  : void
 */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T **y_data, int32_T y_size[2])
{
  k_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y_data, y_size);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const boolean_T u
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const boolean_T u)
{
  const mxArray *y;
  const mxArray *m;
  y = NULL;
  m = emlrtCreateLogicalScalar(u);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *vehiclePoses
 *                const char_T *identifier
 *                struct0_T y_data[]
 *                int32_T y_size[2]
 * Return Type  : void
 */
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *vehiclePoses,
  const char_T *identifier, struct0_T y_data[], int32_T y_size[2])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  d_emlrt_marshallIn(sp, emlrtAlias(vehiclePoses), &thisId, y_data, y_size);
  emlrtDestroyArray(&vehiclePoses);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                struct0_T y_data[]
 *                int32_T y_size[2]
 * Return Type  : void
 */
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct0_T y_data[], int32_T y_size[2])
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[2] = { "vehicle_id", "pose" };

  static const int32_T dims[2] = { 1, 256 };

  const boolean_T bv[2] = { false, true };

  int32_T sizes[2];
  int32_T i;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckVsStructR2012b(sp, parentId, u, 2, fieldNames, 2U, dims, &bv[0],
    sizes);
  y_size[0] = sizes[0];
  y_size[1] = sizes[1];
  for (i = 0; i < sizes[1]; i++) {
    thisId.fIdentifier = "vehicle_id";
    y_data[i].vehicle_id = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b
      (sp, u, i, 0, "vehicle_id")), &thisId);
    thisId.fIdentifier = "pose";
    y_data[i].pose = f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 1, "pose")), &thisId);
  }

  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : uint8_T
 */
static uint8_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  uint8_T y;
  y = l_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *vehicleIdList
 *                const char_T *identifier
 *                real_T **y_data
 *                int32_T y_size[2]
 * Return Type  : void
 */
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *vehicleIdList,
  const char_T *identifier, real_T **y_data, int32_T y_size[2])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(sp, emlrtAlias(vehicleIdList), &thisId, y_data, y_size);
  emlrtDestroyArray(&vehicleIdList);
}

/*
 * Arguments    : const emxArray_struct1_T *u
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const emxArray_struct1_T *u)
{
  const mxArray *y;
  int32_T iv[1];
  static const char * sv[5] = { "t", "px", "py", "vx", "vy" };

  int32_T i;
  int32_T b_j0;
  const mxArray *b_y;
  const mxArray *m;
  y = NULL;
  iv[0] = u->size[0];
  emlrtAssign(&y, emlrtCreateStructArray(1, iv, 5, sv));
  emlrtCreateField(y, "t");
  emlrtCreateField(y, "px");
  emlrtCreateField(y, "py");
  emlrtCreateField(y, "vx");
  emlrtCreateField(y, "vy");
  i = 0;
  for (b_j0 = 0; b_j0 < u->size[0U]; b_j0++) {
    b_y = NULL;
    m = emlrtCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
    *(uint64_T *)emlrtMxGetData(m) = u->data[b_j0].t;
    emlrtAssign(&b_y, m);
    emlrtSetFieldR2017b(y, i, "t", b_y, 0);
    b_y = NULL;
    m = emlrtCreateDoubleScalar(u->data[b_j0].px);
    emlrtAssign(&b_y, m);
    emlrtSetFieldR2017b(y, i, "px", b_y, 1);
    b_y = NULL;
    m = emlrtCreateDoubleScalar(u->data[b_j0].py);
    emlrtAssign(&b_y, m);
    emlrtSetFieldR2017b(y, i, "py", b_y, 2);
    b_y = NULL;
    m = emlrtCreateDoubleScalar(u->data[b_j0].vx);
    emlrtAssign(&b_y, m);
    emlrtSetFieldR2017b(y, i, "vx", b_y, 3);
    b_y = NULL;
    m = emlrtCreateDoubleScalar(u->data[b_j0].vy);
    emlrtAssign(&b_y, m);
    emlrtSetFieldR2017b(y, i, "vy", b_y, 4);
    i++;
  }

  return y;
}

/*
 * Arguments    : emxArray_struct1_T **pEmxArray
 * Return Type  : void
 */
static void emxFree_struct1_T(emxArray_struct1_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_struct1_T *)NULL) {
    if (((*pEmxArray)->data != (struct1_T *)NULL) && (*pEmxArray)->canFreeData)
    {
      emlrtFreeMex((*pEmxArray)->data);
    }

    emlrtFreeMex((*pEmxArray)->size);
    emlrtFreeMex(*pEmxArray);
    *pEmxArray = (emxArray_struct1_T *)NULL;
  }
}

/*
 * Arguments    : const emlrtStack *sp
 *                emxArray_struct1_T **pEmxArray
 *                int32_T numDimensions
 *                boolean_T doPush
 * Return Type  : void
 */
static void emxInit_struct1_T(const emlrtStack *sp, emxArray_struct1_T
  **pEmxArray, int32_T numDimensions, boolean_T doPush)
{
  emxArray_struct1_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_struct1_T *)emlrtMallocMex(sizeof(emxArray_struct1_T));
  if (doPush) {
    emlrtPushHeapReferenceStackR2012b(sp, (void *)pEmxArray, (void *)
      &emxFree_struct1_T);
  }

  emxArray = *pEmxArray;
  emxArray->data = (struct1_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : Pose2D
 */
static Pose2D f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  Pose2D y;
  const mxArray *propValues[3];
  emlrtMsgIdentifier thisId;
  const char * propNames[3] = { "x", "y", "yaw" };

  const char * propClasses[3] = { "Pose2D", "Pose2D", "Pose2D" };

  propValues[0] = NULL;
  propValues[1] = NULL;
  propValues[2] = NULL;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckMcosClass2017a(sp, parentId, u, "Pose2D");
  emlrtGetAllProperties(sp, u, 0, 3, propNames, propClasses, propValues);
  thisId.fIdentifier = "x";
  y.x = g_emlrt_marshallIn(sp, emlrtAlias(propValues[0]), &thisId);
  thisId.fIdentifier = "y";
  y.y = g_emlrt_marshallIn(sp, emlrtAlias(propValues[1]), &thisId);
  thisId.fIdentifier = "yaw";
  y.yaw = g_emlrt_marshallIn(sp, emlrtAlias(propValues[2]), &thisId);
  emlrtDestroyArrays(3, (const mxArray **)&propValues);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T
 */
static real_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = m_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *goalPose
 *                const char_T *identifier
 * Return Type  : Pose2D
 */
static Pose2D h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *goalPose,
  const char_T *identifier)
{
  Pose2D y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(goalPose), &thisId);
  emlrtDestroyArray(&goalPose);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *egoVehicleId
 *                const char_T *identifier
 * Return Type  : uint8_T
 */
static uint8_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *egoVehicleId, const char_T *identifier)
{
  uint8_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = e_emlrt_marshallIn(sp, emlrtAlias(egoVehicleId), &thisId);
  emlrtDestroyArray(&egoVehicleId);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *speed
 *                const char_T *identifier
 * Return Type  : real_T
 */
static real_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *speed,
  const char_T *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = g_emlrt_marshallIn(sp, emlrtAlias(speed), &thisId);
  emlrtDestroyArray(&speed);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T **ret_data
 *                int32_T ret_size[2]
 * Return Type  : void
 */
static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T **ret_data, int32_T ret_size[2])
{
  static const int32_T dims[2] = { 1, 256 };

  const boolean_T bv[2] = { false, true };

  int32_T iv[2];
  emlrtCheckVsBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims, &bv[0],
    iv);
  ret_size[0] = iv[0];
  ret_size[1] = iv[1];
  *ret_data = (real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : uint8_T
 */
static uint8_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  uint8_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "uint8", false, 0U, &dims);
  ret = *(uint8_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T
 */
static real_T m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  real_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, &dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const mxArray * const prhs[5]
 *                int32_T nlhs
 *                const mxArray *plhs[2]
 * Return Type  : void
 */
void planTrajectory_api(const mxArray * const prhs[5], int32_T nlhs, const
  mxArray *plhs[2])
{
  emxArray_struct1_T *trajectory_points;
  real_T (*vehicleIdList_data)[256];
  int32_T vehicleIdList_size[2];
  struct0_T vehiclePoses_data[256];
  int32_T vehiclePoses_size[2];
  Pose2D goalPose;
  uint8_T egoVehicleId;
  real_T speed;
  boolean_T isPathValid;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  emxInit_struct1_T(&st, &trajectory_points, 1, true);

  /* Marshall function inputs */
  emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "vehicleIdList", (real_T **)
                   &vehicleIdList_data, vehicleIdList_size);
  c_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "vehiclePoses",
                     vehiclePoses_data, vehiclePoses_size);
  goalPose = h_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "goalPose");
  egoVehicleId = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "egoVehicleId");
  speed = j_emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "speed");

  /* Invoke the target function */
  planTrajectory(*vehicleIdList_data, vehicleIdList_size, vehiclePoses_data,
                 vehiclePoses_size, &goalPose, egoVehicleId, speed,
                 trajectory_points, &isPathValid);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(trajectory_points);
  emxFree_struct1_T(&trajectory_points);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(isPathValid);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void planTrajectory_atexit(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  planTrajectory_xil_terminate();
  planTrajectory_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void planTrajectory_initialize(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void planTrajectory_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_planTrajectory_api.c
 *
 * [EOF]
 */
