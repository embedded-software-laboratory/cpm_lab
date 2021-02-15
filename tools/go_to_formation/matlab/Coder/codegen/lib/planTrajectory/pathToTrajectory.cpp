//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pathToTrajectory.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 15-Feb-2021 16:41:56
//

// Include Files
#include "pathToTrajectory.h"
#include "DubinsBuiltins.h"
#include "DubinsPathSegment.h"
#include "DubinsPathSegmentCodegen.h"
#include "OneDimArrayBehavior.h"
#include "Path.h"
#include "Path1.h"
#include "all.h"
#include "cosd.h"
#include "ismember.h"
#include "planTrajectory.h"
#include "planTrajectory_rtwutil.h"
#include "rt_nonfinite.h"
#include "sind.h"
#include <cmath>
#include <cstring>
#include <string.h>

// Function Declarations
namespace mgen
{
  static unsigned long eml_i64dplus(unsigned long x, double y);
}

// Function Definitions

//
// Arguments    : unsigned long x
//                double y
// Return Type  : unsigned long
//
namespace mgen
{
  static unsigned long eml_i64dplus(unsigned long x, double y)
  {
    unsigned long z;
    double roundedy;
    roundedy = rt_roundd_snf(y);
    if (y >= 0.0) {
      unsigned long qY;
      qY = x + static_cast<unsigned long>(roundedy);
      if (qY < x) {
        qY = MAX_uint64_T;
      }

      if (y < 1.8446744073709552E+19) {
        z = qY;
      } else {
        z = MAX_uint64_T;
      }
    } else if (y < 0.0) {
      unsigned long qY;
      qY = x - static_cast<unsigned long>(std::floor(-y));
      if (qY > x) {
        qY = 0UL;
      }

      z = x - static_cast<unsigned long>(rt_roundd_snf(-y));
      if (z > x) {
        z = 0UL;
      }

      if (-y < 1.8446744073709552E+19) {
        if (y - roundedy == 0.5) {
          z = qY;
        }
      } else {
        z = 0UL;
      }
    } else {
      z = 0UL;
    }

    return z;
  }

  //
  // Arguments    : const driving_Path *refPath
  //                double speed
  //                coder::array<struct1_T, 1U> *trajectory_points
  // Return Type  : void
  //
  void pathToTrajectory(const driving_Path *refPath, double speed, coder::array<
                        struct1_T, 1U> &trajectory_points)
  {
    double kd;
    coder::array<double, 2U> lengths;
    double ndbl;
    coder::array<double, 2U> intermittingPoses;
    coder::array<double, 2U> transitionPoses;
    double transSegmentLengths[100];
    int nm1d2;
    int n;
    coder::array<double, 1U> allSegmentLengths;
    int i;
    int lastIndex;
    int k;
    d_driving_internal_planning_Dub r;
    double s_data[3];
    int s_size[2];
    coder::array<double, 1U> x;
    double transSegmentLengths_data[100];
    coder::array<boolean_T, 2U> checkPose;
    coder::array<boolean_T, 1U> isTransPose;
    double motionLengths_data[3];
    int nPoses;
    signed char tmp_data[3];
    static const unsigned long t = 0UL;
    signed char ii_data_idx_0;
    unsigned long qY;

    //  single trajectory point
    //  Additionally to transition poses of path segments
    //  interpolate intermitting poses for more exact translation into spline.
    kd = refPath->get_Length();
    if (rtIsNaN(kd)) {
      lengths.set_size(1, 1);
      lengths[0] = rtNaN;
    } else if (kd < 0.0) {
      lengths.set_size(1, 0);
    } else if (rtIsInf(kd) && (0.0 == kd)) {
      lengths.set_size(1, 1);
      lengths[0] = rtNaN;
    } else {
      double apnd;
      double cdiff;
      ndbl = std::floor(kd / 0.5 + 0.5);
      apnd = ndbl * 0.5;
      cdiff = apnd - kd;
      if (std::abs(cdiff) < 4.4408920985006262E-16 * kd) {
        ndbl++;
        apnd = kd;
      } else if (cdiff > 0.0) {
        apnd = (ndbl - 1.0) * 0.5;
      } else {
        ndbl++;
      }

      if (ndbl >= 0.0) {
        n = static_cast<int>(ndbl);
      } else {
        n = 0;
      }

      lengths.set_size(1, n);
      if (n > 0) {
        lengths[0] = 0.0;
        if (n > 1) {
          lengths[n - 1] = apnd;
          nm1d2 = (n - 1) / 2;
          for (k = 0; k <= nm1d2 - 2; k++) {
            kd = (static_cast<double>(k) + 1.0) * 0.5;
            lengths[k + 1] = kd;
            lengths[(n - k) - 2] = apnd - kd;
          }

          if (nm1d2 << 1 == n - 1) {
            lengths[nm1d2] = apnd / 2.0;
          } else {
            kd = static_cast<double>(nm1d2) * 0.5;
            lengths[nm1d2] = kd;
            lengths[nm1d2 + 1] = apnd - kd;
          }
        }
      }
    }

    //  Path length between two intermitting poses.
    refPath->interpolate(lengths, intermittingPoses);
    refPath->interpolate(transitionPoses);

    //     %% Calculate path lengths between transition and intermitting poses
    std::memset(&transSegmentLengths[0], 0, 100U * sizeof(double));
    nm1d2 = intermittingPoses.size(0);
    if (nm1d2 <= 3) {
      nm1d2 = 3;
    }

    if (intermittingPoses.size(0) == 0) {
      n = 0;
    } else {
      n = nm1d2;
    }

    allSegmentLengths.set_size(n);
    for (i = 0; i < n; i++) {
      allSegmentLengths[i] = 0.0;
    }

    lastIndex = 0;
    i = static_cast<int>(refPath->PathSegments.length());
    for (k = 0; k < i; k++) {
      boolean_T exitg1;
      r = refPath->PathSegments;
      r.parenReference((static_cast<double>(k) + 1.0));
      r.get_MotionLengths(s_data, s_size);
      n = 0;
      if (s_data[0] != 0.0) {
        n = 1;
      }

      if (s_data[1] != 0.0) {
        n++;
      }

      if (s_data[2] != 0.0) {
        n++;
      }

      nm1d2 = -1;
      if (s_data[0] != 0.0) {
        nm1d2 = 0;
        motionLengths_data[0] = s_data[0];
      }

      if (s_data[1] != 0.0) {
        nm1d2++;
        motionLengths_data[nm1d2] = s_data[1];
      }

      if (s_data[2] != 0.0) {
        nm1d2++;
        motionLengths_data[nm1d2] = s_data[2];
      }

      nm1d2 = n - 1;
      for (nPoses = 0; nPoses <= nm1d2; nPoses++) {
        tmp_data[nPoses] = static_cast<signed char>(nPoses + lastIndex);
      }

      for (nPoses = 0; nPoses < n; nPoses++) {
        transSegmentLengths[tmp_data[nPoses]] = motionLengths_data[nPoses];
      }

      nm1d2 = 100;
      exitg1 = false;
      while ((!exitg1) && (nm1d2 > 0)) {
        if (transSegmentLengths[nm1d2 - 1] != 0.0) {
          ii_data_idx_0 = static_cast<signed char>(nm1d2);
          exitg1 = true;
        } else {
          nm1d2--;
        }
      }

      lastIndex = ii_data_idx_0;
    }

    n = 0;
    nm1d2 = -1;
    for (k = 0; k < 100; k++) {
      if (transSegmentLengths[k] != 0.0) {
        n++;
        nm1d2++;
        transSegmentLengths_data[nm1d2] = transSegmentLengths[k];
      }
    }

    x.set_size(n);
    for (i = 0; i < n; i++) {
      x[i] = transSegmentLengths_data[i];
    }

    if ((n != 1) && (n != 0) && (n != 1)) {
      for (k = 0; k <= n - 2; k++) {
        x[k + 1] = x[k] + x[k + 1];
      }
    }

    local_ismember(intermittingPoses, transitionPoses, checkPose);
    all(checkPose, isTransPose);

    //  Merge transition poses with intermitting poses into vector allSegmentLengths 
    allSegmentLengths[0] = 0.0;
    if (intermittingPoses.size(0) == 0) {
      n = -3;
    } else if (intermittingPoses.size(0) > 3) {
      n = intermittingPoses.size(0) - 3;
    } else {
      n = 0;
    }

    for (nPoses = 0; nPoses <= n; nPoses++) {
      if (isTransPose[nPoses + 1]) {
        nm1d2 = nPoses + 2;
        lastIndex = isTransPose[0];
        for (k = 2; k <= nm1d2; k++) {
          lastIndex += isTransPose[k - 1];
        }

        allSegmentLengths[nPoses + 1] = x[lastIndex - 2];
      } else {
        nm1d2 = nPoses + 2;
        lastIndex = isTransPose[0];
        for (k = 2; k <= nm1d2; k++) {
          lastIndex += isTransPose[k - 1];
        }

        allSegmentLengths[nPoses + 1] = ((static_cast<double>(nPoses) + 2.0) -
          static_cast<double>(lastIndex)) * 0.5;
      }
    }

    allSegmentLengths[allSegmentLengths.size(0) - 1] = x[x.size(0) - 1];

    //     %% Set trajectory points from transition and intermitting poses
    // TODO: smoothing of start and stop process.
    trajectory_points.set_size((allSegmentLengths.size(0) + 1));
    nm1d2 = allSegmentLengths.size(0) + 1;
    for (i = 0; i < nm1d2; i++) {
      trajectory_points[i].t = t;
      trajectory_points[i].px = 0.0;
      trajectory_points[i].py = 0.0;
      trajectory_points[i].vx = 0.0;
      trajectory_points[i].vy = 0.0;
    }

    // First trajectory point
    trajectory_points[0].px = intermittingPoses[0];
    trajectory_points[0].py = intermittingPoses[intermittingPoses.size(0)];
    trajectory_points[0].vx = 0.0;
    trajectory_points[0].vy = 0.0;
    trajectory_points[0].t = 1000000000UL;

    // [ns]
    //  Second trajectory point
    trajectory_points[1].px = intermittingPoses[0];
    trajectory_points[1].py = intermittingPoses[intermittingPoses.size(0)];
    kd = intermittingPoses[intermittingPoses.size(0) * 2];
    b_cosd(&kd);
    trajectory_points[1].vx = kd * speed;
    kd = intermittingPoses[intermittingPoses.size(0) * 2];
    b_sind(&kd);
    trajectory_points[1].vy = kd * speed;
    qY = trajectory_points[0].t + 1000000000UL;
    if (qY < trajectory_points[0].t) {
      qY = MAX_uint64_T;
    }

    trajectory_points[1].t = qY;

    // [ns]
    i = allSegmentLengths.size(0);
    for (nm1d2 = 0; nm1d2 <= i - 3; nm1d2++) {
      trajectory_points[nm1d2 + 2].px = intermittingPoses[nm1d2 + 1];

      //  [m]
      trajectory_points[nm1d2 + 2].py = intermittingPoses[(nm1d2 +
        intermittingPoses.size(0)) + 1];

      //  [m]
      kd = intermittingPoses[(nm1d2 + intermittingPoses.size(0) * 2) + 1];
      ndbl = kd;
      b_cosd(&ndbl);
      trajectory_points[nm1d2 + 2].vx = ndbl * speed;

      //  [m/s]
      b_sind(&kd);
      trajectory_points[nm1d2 + 2].vy = kd * speed;

      //  [m/s]
      trajectory_points[nm1d2 + 2].t = eml_i64dplus(trajectory_points[1].t,
        allSegmentLengths[nm1d2 + 1] / speed * 1.0E+9);

      //  [ns]
    }

    // Last TrajectoryPoint - correct home position guaranteed?
    trajectory_points[trajectory_points.size(0) - 1].px =
      intermittingPoses[intermittingPoses.size(0) - 1];
    trajectory_points[trajectory_points.size(0) - 1].py = intermittingPoses
      [(intermittingPoses.size(0) + intermittingPoses.size(0)) - 1];
    trajectory_points[trajectory_points.size(0) - 1].vx = 0.0;
    trajectory_points[trajectory_points.size(0) - 1].vy = 0.0;
    trajectory_points[trajectory_points.size(0) - 1].t = eml_i64dplus
      (trajectory_points[1].t, allSegmentLengths[allSegmentLengths.size(0) - 1] /
       speed * 1.0E+9);

    // [ns]
  }
}

//
// File trailer for pathToTrajectory.cpp
//
// [EOF]
//
