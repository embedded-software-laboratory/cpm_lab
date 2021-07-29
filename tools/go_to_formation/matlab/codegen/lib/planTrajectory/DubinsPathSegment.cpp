//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: DubinsPathSegment.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 31-Mar-2021 23:01:38
//

// Include Files
#include "DubinsPathSegment.h"
#include "DubinsBuiltins.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include "unique.h"
#include <string.h>

// Function Definitions

//
// Arguments    : void
// Return Type  : double
//
namespace mgen
{
  double c_driving_internal_planning_Dub::get_Length() const
  {
    return (this->MotionLengths[0] + this->MotionLengths[1]) +
      this->MotionLengths[2];
  }

  //
  // Arguments    : const coder::array<double, 1U> &varargin_1
  //                boolean_T varargin_2
  //                coder::array<double, 2U> &poses
  //                coder::array<double, 1U> &directions
  // Return Type  : void
  //
  void c_driving_internal_planning_Dub::interpolateInternal(const coder::array<
    double, 1U> &varargin_1, boolean_T varargin_2, coder::array<double, 2U>
    &poses, coder::array<double, 1U> &directions) const
  {
    boolean_T x_data[6];
    boolean_T y;
    int k;
    boolean_T exitg1;
    coder::array<double, 1U> a;
    coder::array<double, 1U> samples;
    signed char motionTypesMap[3];
    signed char tmp_data[3];
    signed char b_tmp_data[3];
    coder::array<double, 2U> b_samples;
    unsigned int b_motionTypesMap[3];
    x_data[0] = rtIsNaN(this->MotionLengths[0]);
    x_data[1] = rtIsNaN(this->MotionLengths[1]);
    x_data[2] = rtIsNaN(this->MotionLengths[2]);
    y = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= 2)) {
      if (!x_data[k]) {
        k++;
      } else {
        y = true;
        exitg1 = true;
      }
    }

    if (y) {
      poses.set_size(0, 3);
      directions.set_size(0);
    } else {
      int i;
      int trueCount;
      boolean_T unnamed_idx_0;
      boolean_T unnamed_idx_1;
      boolean_T unnamed_idx_2;
      a.set_size(varargin_1.size(0));
      k = varargin_1.size(0);
      for (i = 0; i < k; i++) {
        a[i] = varargin_1[i];
      }

      if (!varargin_2) {
        a.set_size((varargin_1.size(0) + 1));
        a[0] = 0.0;
        k = varargin_1.size(0);
        for (i = 0; i < k; i++) {
          a[i + 1] = varargin_1[i];
        }
      }

      b_unique_vector(a, samples);
      trueCount = 0;
      motionTypesMap[0] = 1;
      y = false;
      unnamed_idx_0 = false;
      if (!(this->MotionTypes[0] != 'R')) {
        y = true;
        unnamed_idx_0 = true;
      }

      if (y) {
        trueCount = 1;
      }

      motionTypesMap[1] = 1;
      y = false;
      unnamed_idx_1 = false;
      if (!(this->MotionTypes[1] != 'R')) {
        y = true;
        unnamed_idx_1 = true;
      }

      if (y) {
        trueCount++;
      }

      motionTypesMap[2] = 1;
      y = false;
      unnamed_idx_2 = false;
      if (!(this->MotionTypes[2] != 'R')) {
        y = true;
        unnamed_idx_2 = true;
      }

      if (y) {
        trueCount++;
      }

      k = 0;
      if (unnamed_idx_0) {
        tmp_data[0] = 1;
        k = 1;
      }

      if (unnamed_idx_1) {
        tmp_data[k] = 2;
        k++;
      }

      if (unnamed_idx_2) {
        tmp_data[k] = 3;
      }

      for (i = 0; i < trueCount; i++) {
        motionTypesMap[tmp_data[i] - 1] = 2;
      }

      trueCount = 0;
      y = false;
      unnamed_idx_0 = false;
      if (!(this->MotionTypes[0] != 'S')) {
        y = true;
        unnamed_idx_0 = true;
      }

      if (y) {
        trueCount = 1;
      }

      y = false;
      unnamed_idx_1 = false;
      if (!(this->MotionTypes[1] != 'S')) {
        y = true;
        unnamed_idx_1 = true;
      }

      if (y) {
        trueCount++;
      }

      y = false;
      unnamed_idx_2 = false;
      if (!(this->MotionTypes[2] != 'S')) {
        y = true;
        unnamed_idx_2 = true;
      }

      if (y) {
        trueCount++;
      }

      k = 0;
      if (unnamed_idx_0) {
        b_tmp_data[0] = 1;
        k = 1;
      }

      if (unnamed_idx_1) {
        b_tmp_data[k] = 2;
        k++;
      }

      if (unnamed_idx_2) {
        b_tmp_data[k] = 3;
      }

      for (i = 0; i < trueCount; i++) {
        motionTypesMap[b_tmp_data[i] - 1] = 3;
      }

      b_samples.set_size(1, samples.size(0));
      k = samples.size(0);
      for (i = 0; i < k; i++) {
        b_samples[i] = samples[i];
      }

      b_motionTypesMap[0] = motionTypesMap[0] - 1U;
      b_motionTypesMap[1] = motionTypesMap[1] - 1U;
      b_motionTypesMap[2] = motionTypesMap[2] - 1U;
      DubinsBuiltins::c_autonomousDubinsInterpolateSe((this->StartPoseInternal),
        (this->GoalPoseInternal), (b_samples), (this->MinTurningRadius),
        (this->MotionLengths), (b_motionTypesMap), (poses));
      directions.set_size(samples.size(0));
      k = samples.size(0);
      for (i = 0; i < k; i++) {
        directions[i] = 1.0;
      }
    }
  }
}

//
// File trailer for DubinsPathSegment.cpp
//
// [EOF]
//
