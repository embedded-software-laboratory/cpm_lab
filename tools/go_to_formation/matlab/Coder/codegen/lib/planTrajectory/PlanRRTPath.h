//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: PlanRRTPath.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Mar-2021 12:18:40
//
#ifndef PLANRRTPATH_H
#define PLANRRTPATH_H

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
  extern void PlanRRTPath(const Pose2D startPose, const Pose2D goalPose,
    c_driving_internal_costmap_Vehi *costmap, driving_Path *refPath, boolean_T
    *isPathValid);
}

#endif

//
// File trailer for PlanRRTPath.h
//
// [EOF]
//
