//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: PlanRRTPath.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jan-2021 13:31:05
//

// Include Files
#include "PlanRRTPath.h"
#include "DubinsBuiltins.h"
#include "DubinsPathSegment.h"
#include "OneDimArrayBehavior.h"
#include "Path.h"
#include "Path1.h"
#include "VehicleCostmapImpl.h"
#include "checkPathValidity.h"
#include "pathPlannerRRT.h"
#include "pathToTrajectory.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions

//
// %% Path Planning
// Arguments    : const mgen::Pose2D startPose
//                const mgen::Pose2D goalPose
//                mgen::c_driving_internal_costmap_Vehi *costmap
//                mgen::driving_Path *refPath
//                boolean_T *isPathValid
// Return Type  : void
//
namespace mgen
{
  void PlanRRTPath(const Pose2D startPose, const Pose2D goalPose,
                   c_driving_internal_costmap_Vehi *costmap, driving_Path
                   *refPath, boolean_T *isPathValid)
  {
    static pathPlannerRRT planner;
    e_matlabshared_planning_interna lobj_1;
    c_matlabshared_planning_interna lobj_2;
    static c_driving_internal_costmap_Vehi lobj_3;
    double b_startPose[3];
    double b_goalPose[3];
    coder::array<double, 2U> samples;
    coder::array<double, 2U> refPoses;
    coder::array<double, 1U> r;
    coder::array<boolean_T, 1U> x;
    planner.init(costmap, (&lobj_1), (&lobj_2), (&lobj_3));
    b_startPose[0] = startPose.x;
    b_startPose[1] = startPose.y;
    b_startPose[2] = startPose.yaw;
    b_goalPose[0] = goalPose.x;
    b_goalPose[1] = goalPose.y;
    b_goalPose[2] = goalPose.yaw;
    planner.plan(b_startPose, b_goalPose, refPath);
    if (refPath->PathSegments.isempty()) {
      *isPathValid = false;
    } else {
      int loop_ub;
      int i;
      computeSamples(refPath, samples);
      refPath->interpolate(samples, refPoses);
      loop_ub = refPoses.size(0) - 1;
      r.set_size(refPoses.size(0));
      for (i = 0; i <= loop_ub; i++) {
        r[i] = 0.017453292519943295 * refPoses[i + refPoses.size(0) * 2];
      }

      loop_ub = r.size(0);
      for (i = 0; i < loop_ub; i++) {
        refPoses[i + refPoses.size(0) * 2] = r[i];
      }

      if (refPoses.size(0) != 0) {
        boolean_T y;
        boolean_T exitg1;
        costmap->c_checkFreePoses(refPoses, x);
        y = true;
        loop_ub = 1;
        exitg1 = false;
        while ((!exitg1) && (loop_ub <= x.size(0))) {
          if (!x[loop_ub - 1]) {
            y = false;
            exitg1 = true;
          } else {
            loop_ub++;
          }
        }

        if (y) {
          *isPathValid = true;
        } else {
          *isPathValid = false;
        }
      } else {
        *isPathValid = false;
      }
    }
  }
}

//
// File trailer for PlanRRTPath.cpp
//
// [EOF]
//
