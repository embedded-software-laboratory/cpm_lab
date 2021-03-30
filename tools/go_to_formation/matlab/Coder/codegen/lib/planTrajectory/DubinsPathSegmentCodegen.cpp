//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: DubinsPathSegmentCodegen.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Mar-2021 12:18:40
//

// Include Files
#include "DubinsPathSegmentCodegen.h"
#include "DubinsConnection.h"
#include "DubinsPathSegment.h"
#include "DubinsPathSegmentImpl.h"
#include "OneDimArrayBehavior.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions

//
// Arguments    : const mgen::e_driving_internal_planning_Dub *varargin_1
//                mgen::d_driving_internal_planning_Dub *b_this
// Return Type  : void
//
namespace mgen
{
  void d_driving_internal_planning_Dub::create(const
    e_driving_internal_planning_Dub *varargin_1, d_driving_internal_planning_Dub
    *b_this)
  {
    c_driving_internal_planning_Dub r;
    c_driving_internal_planning_Dub::create((varargin_1), (&r));
    b_this->Data.set_size(1, 1);
    b_this->Data[0] = r;
  }

  //
  // Arguments    : const e_driving_internal_planning_Dub *varargin_1
  //                const double varargin_2[3]
  //                const double varargin_3[3]
  //                d_driving_internal_planning_Dub *b_this
  // Return Type  : void
  //
  void d_driving_internal_planning_Dub::create(const
    e_driving_internal_planning_Dub *varargin_1, const double varargin_2[3],
    const double varargin_3[3], d_driving_internal_planning_Dub *b_this)
  {
    c_driving_internal_planning_Dub r;
    c_driving_internal_planning_Dub::create((varargin_1), (varargin_2),
      (varargin_3), (&r));
    b_this->Data.set_size(1, 1);
    b_this->Data[0] = r;
  }

  //
  // Arguments    : double len_data[]
  //                int len_size[1]
  // Return Type  : void
  //
  void d_driving_internal_planning_Dub::get_Length(double len_data[], int
    len_size[1]) const
  {
    len_size[0] = 1;
    len_data[0] = this->Data[0].get_Length();
  }

  //
  // Arguments    : coder::array<double, 1U> &len
  // Return Type  : void
  //
  void d_driving_internal_planning_Dub::get_Length(coder::array<double, 1U> &len)
    const
  {
    double numObj;
    numObj = this->numel();
    if (numObj == 0.0) {
      len.set_size(0);
    } else {
      int i;
      i = static_cast<int>(numObj);
      len.set_size(i);
      for (int n = 0; n < i; n++) {
        len[n] = this->Data[n].get_Length();
      }
    }
  }

  //
  // Arguments    : double motionLens_data[]
  //                int motionLens_size[2]
  // Return Type  : void
  //
  void d_driving_internal_planning_Dub::get_MotionLengths(double
    motionLens_data[], int motionLens_size[2]) const
  {
    motionLens_size[0] = 1;
    motionLens_size[1] = 3;
    motionLens_data[0] = this->Data[0].MotionLengths[0];
    motionLens_data[1] = this->Data[0].MotionLengths[1];
    motionLens_data[2] = this->Data[0].MotionLengths[2];
  }

  //
  // Arguments    : double startPose_data[]
  //                int startPose_size[2]
  // Return Type  : void
  //
  void d_driving_internal_planning_Dub::get_StartPose(double startPose_data[],
    int startPose_size[2]) const
  {
    startPose_size[0] = 1;
    startPose_size[1] = 3;
    this->Data[0].get_StartPose((*(double (*)[3])&startPose_data[0]));
  }

  //
  // Arguments    : const coder::array<double, 1U> &varargin_1
  //                boolean_T varargin_2
  //                coder::array<double, 2U> &varargout_1
  //                coder::array<double, 1U> &varargout_2
  // Return Type  : void
  //
  void d_driving_internal_planning_Dub::interpolateInternal(const coder::array<
    double, 1U> &varargin_1, boolean_T varargin_2, coder::array<double, 2U>
    &varargout_1, coder::array<double, 1U> &varargout_2) const
  {
    this->Data[0].interpolateInternal(varargin_1, varargin_2, varargout_1,
      varargout_2);
  }

  //
  // Arguments    : d_driving_internal_planning_Dub *e
  // Return Type  : void
  //
  void d_driving_internal_planning_Dub::makeempty
    (d_driving_internal_planning_Dub *e)
  {
    e_driving_internal_planning_Dub lobj_0;
    d_driving_internal_planning_Dub::create(e_driving_internal_planning_Dub::
      create((&lobj_0)), (e));
    e->repmat();
  }
}

//
// File trailer for DubinsPathSegmentCodegen.cpp
//
// [EOF]
//
