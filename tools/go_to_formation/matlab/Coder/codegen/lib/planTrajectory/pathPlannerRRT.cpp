//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pathPlannerRRT.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Mar-2021 12:18:40
//

// Include Files
#include "pathPlannerRRT.h"
#include "DubinsBuiltins.h"
#include "DubinsConnection.h"
#include "DubinsConnection1.h"
#include "DubinsPathSegment.h"
#include "DubinsPathSegmentCodegen.h"
#include "OneDimArrayBehavior.h"
#include "Path.h"
#include "Path1.h"
#include "RRTPlanner.h"
#include "VehicleCostmapCodegen.h"
#include "VehicleCostmapImpl.h"
#include "angleUtilities.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions

//
// Arguments    : const coder::array<double, 2U> &pathPoses
//                mgen::driving_Path *refPath
// Return Type  : void
//
namespace mgen
{
  void pathPlannerRRT::createPath(const coder::array<double, 2U> &pathPoses,
    driving_Path *refPath) const
  {
    e_driving_internal_planning_Dub conn;
    d_driving_internal_planning_Dub pathSeg;
    int i;
    double b_pathPoses[3];
    double c_pathPoses[3];
    d_driving_internal_planning_Dub r;
    e_driving_internal_planning_Dub::create((&conn));
    conn.set_MinTurningRadius(this->get_MinTurningRadius());
    d_driving_internal_planning_Dub::makeempty((&pathSeg));
    i = pathPoses.size(0);
    for (int n = 0; n <= i - 2; n++) {
      b_pathPoses[0] = pathPoses[n];
      c_pathPoses[0] = pathPoses[n + 1];
      b_pathPoses[1] = pathPoses[n + pathPoses.size(0)];
      c_pathPoses[1] = pathPoses[(n + pathPoses.size(0)) + 1];
      b_pathPoses[2] = pathPoses[n + pathPoses.size(0) * 2];
      c_pathPoses[2] = pathPoses[(n + pathPoses.size(0) * 2) + 1];
      d_driving_internal_planning_Dub::create((&conn), (b_pathPoses),
        (c_pathPoses), (&r));
      pathSeg.parenAssign((&r), (static_cast<double>(n) + 1.0));
    }

    driving_Path::create((&pathSeg), (refPath));
  }

  //
  // Arguments    : void
  // Return Type  : double
  //
  double pathPlannerRRT::get_MinTurningRadius() const
  {
    return this->InternalPlanner.ConnectionMechanism->TurningRadius;
  }

  //
  // Arguments    : const double startPose[3]
  //                const double goalPose[3]
  // Return Type  : void
  //
  void pathPlannerRRT::validatePoses(const double startPose[3], const double
    goalPose[3]) const
  {
    double poses[6];
    boolean_T unusedExpr[2];
    poses[0] = startPose[0];
    poses[1] = goalPose[0];
    poses[2] = startPose[1];
    poses[3] = goalPose[1];
    poses[4] = 0.017453292519943295 * startPose[2];
    poses[5] = 0.017453292519943295 * goalPose[2];
    this->Costmap->b_checkFreePoses(poses, unusedExpr);
  }

  //
  // Arguments    : c_driving_internal_costmap_Vehi *varargin_1
  //                e_matlabshared_planning_interna *iobj_0
  //                c_matlabshared_planning_interna *iobj_1
  //                c_driving_internal_costmap_Vehi *iobj_2
  // Return Type  : pathPlannerRRT *
  //
  pathPlannerRRT *pathPlannerRRT::init(c_driving_internal_costmap_Vehi
    *varargin_1, e_matlabshared_planning_interna *iobj_0,
    c_matlabshared_planning_interna *iobj_1, c_driving_internal_costmap_Vehi
    *iobj_2)
  {
    pathPlannerRRT *this_;
    static const double dv[3] = { 0.01, 0.01, 0.05235987755982989 };

    this_ = this;
    this_->Costmap = varargin_1->copy(iobj_2);
    this_->InternalPlanner.init(this_->Costmap, dv, iobj_0, iobj_1);
    return this_;
  }

  //
  // Arguments    : double startPose[3]
  //                double goalPose[3]
  //                driving_Path *varargout_1
  // Return Type  : void
  //
  void pathPlannerRRT::plan(double startPose[3], double goalPose[3],
    driving_Path *varargout_1)
  {
    d_driving_internal_planning_Dub pathSeg;
    coder::array<double, 2U> pathPoses;
    int loop_ub;
    coder::array<double, 1U> r;
    int i;
    driving_Path r1;
    this->validatePoses(startPose, goalPose);
    angleUtilities::convertAndWrapTo2Pi((&startPose[2]));
    angleUtilities::convertAndWrapTo2Pi((&goalPose[2]));
    d_driving_internal_planning_Dub::makeempty((&pathSeg));
    driving_Path::create((&pathSeg), (&this->Path));
    this->InternalPlanner.planPath(startPose, goalPose, pathPoses);
    loop_ub = pathPoses.size(0) - 1;
    r.set_size(pathPoses.size(0));
    for (i = 0; i <= loop_ub; i++) {
      r[i] = 57.295779513082323 * pathPoses[i + pathPoses.size(0) * 2];
    }

    loop_ub = r.size(0);
    for (i = 0; i < loop_ub; i++) {
      pathPoses[i + pathPoses.size(0) * 2] = r[i];
    }

    this->createPath(pathPoses, (&r1));
    this->Path = r1;
    *varargout_1 = this->Path;
  }
}

//
// File trailer for pathPlannerRRT.cpp
//
// [EOF]
//
