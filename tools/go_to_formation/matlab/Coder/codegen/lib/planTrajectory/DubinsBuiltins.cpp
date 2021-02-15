//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: DubinsBuiltins.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 15-Feb-2021 16:41:56
//

// Include Files
#include "DubinsBuiltins.h"
#include "DubinsPathSegment.h"
#include "autonomouscodegen_dubins_api.hpp"
#include "autonomouscodegen_dubins_tbb_api.hpp"
#include "pathToTrajectory.h"
#include "planTrajectory.h"
#include "planTrajectory_rtwutil.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions

//
// Arguments    : const double startPose[3]
//                const double goalPose[3]
//                double turningRadius
// Return Type  : double
//
namespace mgen
{
  double DubinsBuiltins::autonomousDubinsDistance(const double startPose[3],
    const double goalPose[3], double turningRadius)
  {
    double dist;
    autonomousDubinsDistanceCodegen_tbb_real64(startPose, 1U, goalPose, 1U,
      turningRadius, &dist);
    return dist;
  }

  //
  // Arguments    : const double startPose[3]
  //                const coder::array<double, 2U> &goalPose
  //                double turningRadius
  //                coder::array<double, 1U> &dist
  // Return Type  : void
  //
  void DubinsBuiltins::autonomousDubinsDistance(const double startPose[3], const
    coder::array<double, 2U> &goalPose, double turningRadius, coder::array<
    double, 1U> &dist)
  {
    int u1;
    u1 = goalPose.size(0);
    if (1 > u1) {
      u1 = 1;
    }

    dist.set_size(u1);
    autonomousDubinsDistanceCodegen_tbb_real64(startPose, 1U, &goalPose[0],
      static_cast<unsigned int>(goalPose.size(0)), turningRadius, &(dist.data())
      [0]);
  }

  //
  // Arguments    : const coder::array<double, 2U> &startPose
  //                const double goalPose[3]
  //                double turningRadius
  //                coder::array<double, 1U> &dist
  // Return Type  : void
  //
  void DubinsBuiltins::autonomousDubinsDistance(const coder::array<double, 2U>
    &startPose, const double goalPose[3], double turningRadius, coder::array<
    double, 1U> &dist)
  {
    int u0;
    u0 = startPose.size(0);
    if (u0 <= 1) {
      u0 = 1;
    }

    dist.set_size(u0);
    autonomousDubinsDistanceCodegen_tbb_real64(&startPose[0], static_cast<
      unsigned int>(startPose.size(0)), goalPose, 1U, turningRadius, &(dist.data
      ())[0]);
  }

  //
  // Arguments    : const coder::array<double, 2U> &startPose
  //                const double goalPose_data[]
  //                const int goalPose_size[2]
  //                double turningRadius
  //                coder::array<double, 1U> &dist
  // Return Type  : void
  //
  void DubinsBuiltins::autonomousDubinsDistance(const coder::array<double, 2U>
    &startPose, const double goalPose_data[], const int goalPose_size[2], double
    turningRadius, coder::array<double, 1U> &dist)
  {
    int u0;
    int u1;
    u0 = startPose.size(0);
    u1 = goalPose_size[0];
    if (u0 > u1) {
      u1 = u0;
    }

    dist.set_size(u1);
    autonomousDubinsDistanceCodegen_tbb_real64(&startPose[0], static_cast<
      unsigned int>(startPose.size(0)), &goalPose_data[0], static_cast<unsigned
      int>(goalPose_size[0]), turningRadius, &(dist.data())[0]);
  }

  //
  // Arguments    : const double startPose[3]
  //                const double goalPose_data[]
  //                double connectionDistance
  //                double numSteps
  //                double turningRadius
  //                coder::array<double, 2U> &poses
  // Return Type  : void
  //
  void DubinsBuiltins::autonomousDubinsInterpolate(const double startPose[3],
    const double goalPose_data[], double connectionDistance, double numSteps,
    double turningRadius, coder::array<double, 2U> &poses)
  {
    double d;
    unsigned int u;
    poses.set_size((static_cast<int>(numSteps + 2.0)), 3);
    d = rt_roundd_snf(numSteps);
    if (d < 4.294967296E+9) {
      if (d >= 0.0) {
        u = static_cast<unsigned int>(d);
      } else {
        u = 0U;
      }
    } else if (d >= 4.294967296E+9) {
      u = MAX_uint32_T;
    } else {
      u = 0U;
    }

    autonomousDubinsInterpolateCodegen_real64(startPose, &goalPose_data[0],
      connectionDistance, u, turningRadius, &poses[0]);
  }

  //
  // Arguments    : const double startPose[3]
  //                const double goalPose[3]
  //                double turningRadius
  //                const coder::array<cell_wrap_38, 2U> &disabledTypes
  //                double *cost
  //                double motionLengths[3]
  //                double motionTypes[3]
  // Return Type  : void
  //
  void DubinsBuiltins::autonomousDubinsSegments(const double startPose[3], const
    double goalPose[3], double turningRadius, const coder::array<cell_wrap_38,
    2U> &disabledTypes, double *cost, double motionLengths[3], double
    motionTypes[3])
  {
    int ret;
    boolean_T allPathTypes[6];
    int u1;
    cell_wrap_38 r;
    static const char b[3] = { 'L', 'S', 'L' };

    boolean_T match[6];
    static const char b_b[3] = { 'L', 'S', 'R' };

    static const char c_b[3] = { 'R', 'S', 'L' };

    static const char d_b[3] = { 'R', 'S', 'R' };

    static const char e_b[3] = { 'R', 'L', 'R' };

    static const char f_b[3] = { 'L', 'R', 'L' };

    for (ret = 0; ret < 6; ret++) {
      allPathTypes[ret] = true;
    }

    if ((disabledTypes.size(0) == 0) || (disabledTypes.size(1) == 0)) {
      u1 = 0;
    } else {
      ret = disabledTypes.size(0);
      u1 = disabledTypes.size(1);
      if (ret > u1) {
        u1 = ret;
      }
    }

    for (int i = 0; i < u1; i++) {
      boolean_T y;
      boolean_T exitg1;
      r.f1[0] = disabledTypes[i].f1[0];
      r.f1[1] = disabledTypes[i].f1[1];
      r.f1[2] = disabledTypes[i].f1[2];
      ret = memcmp(&r.f1[0], &b[0], 3);
      match[0] = (ret == 0);
      ret = memcmp(&r.f1[0], &b_b[0], 3);
      match[1] = (ret == 0);
      ret = memcmp(&r.f1[0], &c_b[0], 3);
      match[2] = (ret == 0);
      ret = memcmp(&r.f1[0], &d_b[0], 3);
      match[3] = (ret == 0);
      ret = memcmp(&r.f1[0], &e_b[0], 3);
      match[4] = (ret == 0);
      ret = memcmp(&r.f1[0], &f_b[0], 3);
      match[5] = (ret == 0);
      y = false;
      ret = 0;
      exitg1 = false;
      while ((!exitg1) && (ret <= 5)) {
        if (match[ret]) {
          y = true;
          exitg1 = true;
        } else {
          ret++;
        }
      }

      if (y) {
        for (ret = 0; ret < 6; ret++) {
          if (match[ret]) {
            allPathTypes[ret] = false;
          }
        }
      }
    }

    autonomousDubinsSegmentsCodegen_tbb_real64(startPose, 1U, goalPose, 1U,
      turningRadius, allPathTypes, true, 3U, cost, motionLengths, motionTypes);
  }

  //
  // Arguments    : const double startPose[3]
  //                const double goalPose[3]
  //                double connectionDistance
  //                double numSteps
  //                double turningRadius
  //                coder::array<double, 2U> &poses
  // Return Type  : void
  //
  void DubinsBuiltins::b_autonomousDubinsInterpolate(const double startPose[3],
    const double goalPose[3], double connectionDistance, double numSteps, double
    turningRadius, coder::array<double, 2U> &poses)
  {
    double d;
    unsigned int u;
    poses.set_size((static_cast<int>(numSteps + 2.0)), 3);
    d = rt_roundd_snf(numSteps);
    if (d < 4.294967296E+9) {
      if (d >= 0.0) {
        u = static_cast<unsigned int>(d);
      } else {
        u = 0U;
      }
    } else if (d >= 4.294967296E+9) {
      u = MAX_uint32_T;
    } else {
      u = 0U;
    }

    autonomousDubinsInterpolateCodegen_real64(startPose, goalPose,
      connectionDistance, u, turningRadius, &poses[0]);
  }

  //
  // Arguments    : const double startPose[3]
  //                const double goalPose[3]
  //                const coder::array<double, 2U> &samples
  //                double turningRadius
  //                const double segmentsLengths[3]
  //                const unsigned int segmentsTypes[3]
  //                coder::array<double, 2U> &poses
  // Return Type  : void
  //
  void DubinsBuiltins::c_autonomousDubinsInterpolateSe(const double startPose[3],
    const double goalPose[3], const coder::array<double, 2U> &samples, double
    turningRadius, const double segmentsLengths[3], const unsigned int
    segmentsTypes[3], coder::array<double, 2U> &poses)
  {
    poses.set_size(samples.size(1), 3);
    autonomousDubinsInterpolateSegmentsCodegen_real64(startPose, goalPose,
      &samples[0], static_cast<unsigned int>(samples.size(1)), turningRadius,
      segmentsLengths, segmentsTypes, &poses[0]);
  }
}

//
// File trailer for DubinsBuiltins.cpp
//
// [EOF]
//
