//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: checkPathValidity.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jan-2021 13:31:05
//

// Include Files
#include "checkPathValidity.h"
#include "DubinsBuiltins.h"
#include "DubinsPathSegment.h"
#include "DubinsPathSegmentCodegen.h"
#include "OneDimArrayBehavior.h"
#include "Path1.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include "unique.h"
#include <cmath>
#include <string.h>

// Function Definitions

//
// Arguments    : const mgen::driving_Path *path
//                coder::array<double, 2U> *pathSamples
// Return Type  : void
//
namespace mgen
{
  void computeSamples(const driving_Path *path, coder::array<double, 2U>
                      &pathSamples)
  {
    coder::array<double, 1U> segmentLengths;
    int n;
    int idx;
    double longestSegment;
    int k;
    int i;
    double numSteps;
    double kd;
    coder::array<double, 1U> accumLengths;
    coder::array<double, 2U> thisSegmentSamples;
    path->PathSegments.get_Length(segmentLengths);
    n = segmentLengths.size(0);
    if (segmentLengths.size(0) <= 2) {
      if (segmentLengths.size(0) == 1) {
        longestSegment = segmentLengths[0];
      } else if ((segmentLengths[0] < segmentLengths[1]) || (rtIsNaN
                  (segmentLengths[0]) && (!rtIsNaN(segmentLengths[1])))) {
        longestSegment = segmentLengths[1];
      } else {
        longestSegment = segmentLengths[0];
      }
    } else {
      if (!rtIsNaN(segmentLengths[0])) {
        idx = 1;
      } else {
        boolean_T exitg1;
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= segmentLengths.size(0))) {
          if (!rtIsNaN(segmentLengths[k - 1])) {
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (idx == 0) {
        longestSegment = segmentLengths[0];
      } else {
        longestSegment = segmentLengths[idx - 1];
        i = idx + 1;
        for (k = i; k <= n; k++) {
          kd = segmentLengths[k - 1];
          if (longestSegment < kd) {
            longestSegment = kd;
          }
        }
      }
    }

    longestSegment = std::ceil(longestSegment / 0.01);
    if ((3.0 > longestSegment) || rtIsNaN(longestSegment)) {
      longestSegment = 3.0;
    }

    numSteps = 5.0 * longestSegment;
    accumLengths.set_size((segmentLengths.size(0) + 1));
    accumLengths[0] = 0.0;
    idx = segmentLengths.size(0);
    for (i = 0; i < idx; i++) {
      accumLengths[i + 1] = segmentLengths[i];
    }

    if (accumLengths.size(0) != 1) {
      i = accumLengths.size(0);
      for (k = 0; k <= i - 2; k++) {
        accumLengths[k + 1] = accumLengths[k] + accumLengths[k + 1];
      }
    }

    pathSamples.set_size(1, 1);
    pathSamples[0] = 0.0;
    i = static_cast<int>((path->PathSegments.numel() + 1.0) + -1.0);
    for (n = 0; n < i; n++) {
      double step;
      double a;
      int nm1d2;
      kd = accumLengths[n + 1];
      step = (kd - accumLengths[n]) / numSteps;
      a = accumLengths[n] + step;
      if (rtIsNaN(a) || rtIsNaN(step) || rtIsNaN(kd)) {
        thisSegmentSamples.set_size(1, 1);
        thisSegmentSamples[0] = rtNaN;
      } else if ((step == 0.0) || ((a < kd) && (step < 0.0)) || ((kd < a) &&
                  (step > 0.0))) {
        thisSegmentSamples.set_size(1, 0);
      } else if ((rtIsInf(a) || rtIsInf(kd)) && (rtIsInf(step) || (a == kd))) {
        thisSegmentSamples.set_size(1, 1);
        thisSegmentSamples[0] = rtNaN;
      } else if (rtIsInf(step)) {
        thisSegmentSamples.set_size(1, 1);
        thisSegmentSamples[0] = a;
      } else if ((std::floor(a) == a) && (std::floor(step) == step)) {
        idx = static_cast<int>(std::floor((accumLengths[n + 1] - a) / step));
        thisSegmentSamples.set_size(1, (idx + 1));
        for (nm1d2 = 0; nm1d2 <= idx; nm1d2++) {
          thisSegmentSamples[nm1d2] = a + step * static_cast<double>(nm1d2);
        }
      } else {
        double ndbl;
        double apnd;
        double cdiff;
        double u0;
        ndbl = std::floor((kd - a) / step + 0.5);
        apnd = a + ndbl * step;
        if (step > 0.0) {
          cdiff = apnd - kd;
        } else {
          cdiff = kd - apnd;
        }

        u0 = std::abs(a);
        longestSegment = std::abs(kd);
        if ((u0 > longestSegment) || rtIsNaN(longestSegment)) {
          longestSegment = u0;
        }

        if (std::abs(cdiff) < 4.4408920985006262E-16 * longestSegment) {
          ndbl++;
          apnd = kd;
        } else if (cdiff > 0.0) {
          apnd = a + (ndbl - 1.0) * step;
        } else {
          ndbl++;
        }

        if (ndbl >= 0.0) {
          idx = static_cast<int>(ndbl);
        } else {
          idx = 0;
        }

        thisSegmentSamples.set_size(1, idx);
        if (idx > 0) {
          thisSegmentSamples[0] = a;
          if (idx > 1) {
            thisSegmentSamples[idx - 1] = apnd;
            nm1d2 = (idx - 1) / 2;
            for (k = 0; k <= nm1d2 - 2; k++) {
              kd = (static_cast<double>(k) + 1.0) * step;
              thisSegmentSamples[k + 1] = a + kd;
              thisSegmentSamples[(idx - k) - 2] = apnd - kd;
            }

            if (nm1d2 << 1 == idx - 1) {
              thisSegmentSamples[nm1d2] = (a + apnd) / 2.0;
            } else {
              kd = static_cast<double>(nm1d2) * step;
              thisSegmentSamples[nm1d2] = a + kd;
              thisSegmentSamples[nm1d2 + 1] = apnd - kd;
            }
          }
        }
      }

      nm1d2 = pathSamples.size(1);
      idx = thisSegmentSamples.size(1);
      pathSamples.set_size(pathSamples.size(0), (pathSamples.size(1) +
        thisSegmentSamples.size(1)));
      for (k = 0; k < idx; k++) {
        pathSamples[nm1d2 + k] = thisSegmentSamples[k];
      }
    }

    thisSegmentSamples.set_size(1, pathSamples.size(1));
    idx = pathSamples.size(0) * pathSamples.size(1) - 1;
    for (i = 0; i <= idx; i++) {
      thisSegmentSamples[i] = pathSamples[i];
    }

    unique_vector(thisSegmentSamples, pathSamples);
  }
}

//
// File trailer for checkPathValidity.cpp
//
// [EOF]
//
