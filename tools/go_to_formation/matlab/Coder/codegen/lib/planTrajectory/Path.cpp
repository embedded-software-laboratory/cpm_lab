//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Path.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Mar-2021 12:18:40
//

// Include Files
#include "Path.h"
#include "DubinsBuiltins.h"
#include "DubinsPathSegment.h"
#include "DubinsPathSegmentCodegen.h"
#include "OneDimArrayBehavior.h"
#include "Path1.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions

//
// Arguments    : const mgen::d_driving_internal_planning_Dub *varargin_1
//                mgen::driving_Path *b_this
// Return Type  : void
//
namespace mgen
{
  void driving_Path::create(const d_driving_internal_planning_Dub *varargin_1,
    driving_Path *b_this)
  {
    b_this->PathSegments = *varargin_1;
  }

  //
  // Arguments    : double startPose_data[]
  //                int startPose_size[2]
  // Return Type  : void
  //
  void driving_Path::get_StartPose(double startPose_data[], int startPose_size[2])
    const
  {
    d_driving_internal_planning_Dub r;
    double tmp_data[3];
    int tmp_size[2];
    if (this->PathSegments.isempty()) {
      startPose_size[0] = 0;
      startPose_size[1] = 3;
    } else {
      r = this->PathSegments;
      r.parenReference();
      r.get_StartPose(tmp_data, tmp_size);
      startPose_size[0] = 1;
      startPose_size[1] = 3;
      startPose_data[0] = tmp_data[0];
      startPose_data[1] = tmp_data[1];
      startPose_data[2] = tmp_data[2];
    }
  }

  //
  // Arguments    : coder::array<double, 2U> &poses
  // Return Type  : void
  //
  void driving_Path::interpolate(coder::array<double, 2U> &poses) const
  {
    coder::array<double, 1U> directions;
    int loop_ub;
    int i;
    this->interpolate(poses, directions);
    loop_ub = poses.size(0) - 1;
    directions.set_size(poses.size(0));
    for (i = 0; i <= loop_ub; i++) {
      directions[i] = 57.295779513082323 * poses[i + poses.size(0) * 2];
    }

    loop_ub = directions.size(0);
    for (i = 0; i < loop_ub; i++) {
      poses[i + poses.size(0) * 2] = directions[i];
    }
  }

  //
  // Arguments    : const coder::array<double, 2U> &varargin_1
  //                coder::array<double, 2U> &poses
  // Return Type  : void
  //
  void driving_Path::interpolate(const coder::array<double, 2U> &varargin_1,
    coder::array<double, 2U> &poses) const
  {
    coder::array<double, 1U> directions;
    int loop_ub;
    int i;
    this->interpolate(varargin_1, poses, directions);
    loop_ub = poses.size(0) - 1;
    directions.set_size(poses.size(0));
    for (i = 0; i <= loop_ub; i++) {
      directions[i] = 57.295779513082323 * poses[i + poses.size(0) * 2];
    }

    loop_ub = directions.size(0);
    for (i = 0; i < loop_ub; i++) {
      poses[i + poses.size(0) * 2] = directions[i];
    }
  }
}

//
// File trailer for Path.cpp
//
// [EOF]
//
