//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Path1.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Mar-2021 12:18:40
//

// Include Files
#include "Path1.h"
#include "DubinsBuiltins.h"
#include "DubinsPathSegment.h"
#include "DubinsPathSegmentCodegen.h"
#include "OneDimArrayBehavior.h"
#include "Path.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include "unique.h"
#include <string.h>

// Type Definitions
namespace mgen
{
  class Path
  {
   public:
    static void discretizecg(const coder::array<double, 1U> &x, const coder::
      array<double, 1U> &edges, coder::array<double, 1U> &bins);
  };

  struct emxArray_real_T_1x3
  {
    double data[3];
    int size[2];
  };
}

// Function Definitions

//
// Arguments    : const coder::array<double, 1U> &x
//                const coder::array<double, 1U> &edges
//                coder::array<double, 1U> &bins
// Return Type  : void
//
namespace mgen
{
  void Path::discretizecg(const coder::array<double, 1U> &x, const coder::array<
    double, 1U> &edges, coder::array<double, 1U> &bins)
  {
    int numEdges;
    int i;
    bins.set_size(x.size(0));
    numEdges = edges.size(0) - 2;
    i = x.size(0);
    for (int n = 0; n < i; n++) {
      int e;
      boolean_T exitg1;
      e = 0;
      exitg1 = false;
      while ((!exitg1) && (e <= numEdges - 1)) {
        if ((x[n] >= edges[e]) && (x[n] < edges[e + 1])) {
          bins[n] = static_cast<double>(e) + 1.0;
          exitg1 = true;
        } else {
          e++;
        }
      }

      if (x[n] >= edges[numEdges]) {
        bins[n] = numEdges + 1;
      }
    }
  }

  //
  // Arguments    : void
  // Return Type  : double
  //
  double driving_Path::get_Length() const
  {
    double len;
    coder::array<double, 1U> x;
    if (this->PathSegments.isempty()) {
      len = 0.0;
    } else {
      int vlen;
      this->PathSegments.get_Length(x);
      vlen = x.size(0);
      if (x.size(0) == 0) {
        len = 0.0;
      } else {
        len = x[0];
        for (int k = 2; k <= vlen; k++) {
          len += x[k - 1];
        }
      }
    }

    return len;
  }

  //
  // Arguments    : coder::array<double, 2U> &poses
  //                coder::array<double, 1U> &directions
  // Return Type  : void
  //
  void driving_Path::interpolate(coder::array<double, 2U> &poses, coder::array<
    double, 1U> &directions) const
  {
    emxArray_real_T_1x3 b_this;
    d_driving_internal_planning_Dub r;
    coder::array<double, 2U> transitionSamples;
    coder::array<double, 1U> accumLengths;
    coder::array<double, 1U> segmentSamples;
    double motionLens_data[3];
    int motionLens_size[2];
    coder::array<double, 1U> samplesAll;
    coder::array<double, 1U> segmentIndex;
    double segLen_data[1];
    int tmp_size[1];
    coder::array<double, 2U> b_directions;
    coder::array<int, 1U> r1;
    coder::array<double, 2U> segmentPoses;
    coder::array<double, 1U> segmentDirections;
    coder::array<double, 2U> b_poses;
    if (this->PathSegments.isempty()) {
      this->get_StartPose(b_this.data, b_this.size);
      poses.set_size(0, 3);
      this->get_StartPose(b_this.data, b_this.size);
      directions.set_size(0);
    } else {
      double numSegments_tmp;
      unsigned int t;
      double accum;
      int i;
      int s;
      int nx;
      int transitionSamples_tmp;
      numSegments_tmp = this->PathSegments.numel();
      r = this->PathSegments;
      r.parenReference();
      r = this->PathSegments;
      r.parenReference();
      transitionSamples.set_size(1, (static_cast<int>(numSegments_tmp * 3.0)));
      t = 1U;
      accum = 0.0;
      i = static_cast<int>(numSegments_tmp);
      for (s = 0; s < i; s++) {
        r = this->PathSegments;
        r.parenReference((static_cast<double>(s) + 1.0));
        r.get_MotionLengths(motionLens_data, motionLens_size);
        nx = static_cast<int>(t) - 1;
        transitionSamples[nx] = accum + motionLens_data[0];
        t++;
        transitionSamples_tmp = static_cast<int>(t) - 1;
        transitionSamples[transitionSamples_tmp] = transitionSamples[nx] +
          motionLens_data[1];
        t++;
        nx = static_cast<int>(t) - 1;
        transitionSamples[nx] = transitionSamples[transitionSamples_tmp] +
          motionLens_data[2];
        accum = transitionSamples[nx];
        t++;
      }

      nx = transitionSamples.size(1);
      accumLengths = transitionSamples.reshape(nx);
      b_unique_vector(accumLengths, segmentSamples);
      accum = this->get_Length();
      samplesAll.set_size(segmentSamples.size(0));
      nx = segmentSamples.size(0);
      for (transitionSamples_tmp = 0; transitionSamples_tmp < nx;
           transitionSamples_tmp++) {
        if ((segmentSamples[transitionSamples_tmp] < accum) || rtIsNaN(accum)) {
          samplesAll[transitionSamples_tmp] =
            segmentSamples[transitionSamples_tmp];
        } else {
          samplesAll[transitionSamples_tmp] = accum;
        }
      }

      r = this->PathSegments;
      r.parenReference();
      accumLengths.set_size((static_cast<int>(numSegments_tmp + 1.0)));
      accumLengths[0] = 0.0;
      for (s = 0; s < i; s++) {
        r = this->PathSegments;
        r.parenReference((static_cast<double>(s) + 1.0));
        r.get_Length(segLen_data, tmp_size);
        accumLengths[s + 1] = accumLengths[s] + segLen_data[0];
      }

      Path::discretizecg((samplesAll), (accumLengths), (segmentIndex));
      this->get_StartPose(b_this.data, b_this.size);
      poses.set_size(0, 3);
      b_directions.set_size(0, 1);
      for (int n = 0; n < i; n++) {
        int i1;
        transitionSamples_tmp = segmentIndex.size(0) - 1;
        nx = 0;
        for (s = 0; s <= transitionSamples_tmp; s++) {
          if (segmentIndex[s] == static_cast<double>(n) + 1.0) {
            nx++;
          }
        }

        r1.set_size(nx);
        nx = 0;
        for (s = 0; s <= transitionSamples_tmp; s++) {
          if (segmentIndex[s] == static_cast<double>(n) + 1.0) {
            r1[nx] = s + 1;
            nx++;
          }
        }

        nx = r1.size(0);
        segmentSamples.set_size(r1.size(0));
        for (i1 = 0; i1 < nx; i1++) {
          segmentSamples[i1] = samplesAll[r1[i1] - 1] - accumLengths[n];
        }

        r = this->PathSegments;
        r.parenReference((static_cast<double>(n) + 1.0));
        r.get_Length(segLen_data, tmp_size);
        i1 = segmentSamples.size(0);
        for (s = 0; s < i1; s++) {
          if ((segmentSamples[s] < segLen_data[0]) || rtIsNaN(segLen_data[0])) {
            accum = segmentSamples[s];
          } else {
            accum = segLen_data[0];
          }

          segmentSamples[s] = accum;
        }

        r = this->PathSegments;
        r.parenReference((static_cast<double>(n) + 1.0));
        r.interpolateInternal(segmentSamples, (n + 1U > 1U), segmentPoses,
                              segmentDirections);
        nx = poses.size(0);
        b_poses.set_size((poses.size(0) + segmentPoses.size(0)), 3);
        for (i1 = 0; i1 < 3; i1++) {
          for (transitionSamples_tmp = 0; transitionSamples_tmp < nx;
               transitionSamples_tmp++) {
            b_poses[transitionSamples_tmp + b_poses.size(0) * i1] =
              poses[transitionSamples_tmp + poses.size(0) * i1];
          }
        }

        nx = segmentPoses.size(0);
        for (i1 = 0; i1 < 3; i1++) {
          for (transitionSamples_tmp = 0; transitionSamples_tmp < nx;
               transitionSamples_tmp++) {
            b_poses[(transitionSamples_tmp + poses.size(0)) + b_poses.size(0) *
              i1] = segmentPoses[transitionSamples_tmp + segmentPoses.size(0) *
              i1];
          }
        }

        poses.set_size(b_poses.size(0), 3);
        nx = b_poses.size(0) * b_poses.size(1);
        for (i1 = 0; i1 < nx; i1++) {
          poses[i1] = b_poses[i1];
        }

        nx = b_directions.size(0);
        directions.set_size((b_directions.size(0) + segmentDirections.size(0)));
        for (i1 = 0; i1 < nx; i1++) {
          directions[i1] = b_directions[i1];
        }

        nx = segmentDirections.size(0);
        for (i1 = 0; i1 < nx; i1++) {
          directions[i1 + b_directions.size(0)] = segmentDirections[i1];
        }

        b_directions.set_size(directions.size(0), 1);
        nx = directions.size(0);
        for (i1 = 0; i1 < nx; i1++) {
          b_directions[i1] = directions[i1];
        }
      }

      directions.set_size(b_directions.size(0));
      nx = b_directions.size(0);
      for (i = 0; i < nx; i++) {
        directions[i] = b_directions[i];
      }
    }
  }

  //
  // Arguments    : const coder::array<double, 2U> &varargin_1
  //                coder::array<double, 2U> &poses
  //                coder::array<double, 1U> &directions
  // Return Type  : void
  //
  void driving_Path::interpolate(const coder::array<double, 2U> &varargin_1,
    coder::array<double, 2U> &poses, coder::array<double, 1U> &directions) const
  {
    emxArray_real_T_1x3 b_this;
    d_driving_internal_planning_Dub r;
    coder::array<double, 2U> transitionSamples;
    coder::array<double, 1U> accumLengths;
    double motionLens_data[3];
    int motionLens_size[2];
    coder::array<double, 1U> segmentSamples;
    coder::array<double, 1U> samplesAll;
    coder::array<double, 1U> segmentIndex;
    double segLen_data[1];
    int tmp_size[1];
    coder::array<double, 2U> b_directions;
    coder::array<int, 1U> r1;
    coder::array<double, 2U> segmentPoses;
    coder::array<double, 1U> segmentDirections;
    coder::array<double, 2U> b_poses;
    if (this->PathSegments.isempty()) {
      this->get_StartPose(b_this.data, b_this.size);
      poses.set_size(0, 3);
      this->get_StartPose(b_this.data, b_this.size);
      directions.set_size(0);
    } else {
      double numSegments_tmp;
      unsigned int t;
      double accum;
      int i;
      int s;
      int nx;
      int i1;
      int transitionSamples_tmp;
      numSegments_tmp = this->PathSegments.numel();
      r = this->PathSegments;
      r.parenReference();
      r = this->PathSegments;
      r.parenReference();
      transitionSamples.set_size(1, (static_cast<int>(numSegments_tmp * 3.0)));
      t = 1U;
      accum = 0.0;
      i = static_cast<int>(numSegments_tmp);
      for (s = 0; s < i; s++) {
        r = this->PathSegments;
        r.parenReference((static_cast<double>(s) + 1.0));
        r.get_MotionLengths(motionLens_data, motionLens_size);
        nx = static_cast<int>(t) - 1;
        transitionSamples[nx] = accum + motionLens_data[0];
        t++;
        transitionSamples_tmp = static_cast<int>(t) - 1;
        transitionSamples[transitionSamples_tmp] = transitionSamples[nx] +
          motionLens_data[1];
        t++;
        nx = static_cast<int>(t) - 1;
        transitionSamples[nx] = transitionSamples[transitionSamples_tmp] +
          motionLens_data[2];
        accum = transitionSamples[nx];
        t++;
      }

      if (varargin_1.size(1) != 0) {
        accumLengths.set_size((varargin_1.size(1) + transitionSamples.size(1)));
        nx = varargin_1.size(1);
        for (i1 = 0; i1 < nx; i1++) {
          accumLengths[i1] = varargin_1[i1];
        }

        nx = transitionSamples.size(1);
        for (i1 = 0; i1 < nx; i1++) {
          accumLengths[i1 + varargin_1.size(1)] = transitionSamples[i1];
        }

        b_unique_vector(accumLengths, segmentSamples);
        accum = this->get_Length();
        samplesAll.set_size(segmentSamples.size(0));
        nx = segmentSamples.size(0);
        for (transitionSamples_tmp = 0; transitionSamples_tmp < nx;
             transitionSamples_tmp++) {
          if ((segmentSamples[transitionSamples_tmp] < accum) || rtIsNaN(accum))
          {
            samplesAll[transitionSamples_tmp] =
              segmentSamples[transitionSamples_tmp];
          } else {
            samplesAll[transitionSamples_tmp] = accum;
          }
        }
      } else {
        nx = transitionSamples.size(1);
        accumLengths = transitionSamples.reshape(nx);
        b_unique_vector(accumLengths, segmentSamples);
        accum = this->get_Length();
        samplesAll.set_size(segmentSamples.size(0));
        nx = segmentSamples.size(0);
        for (transitionSamples_tmp = 0; transitionSamples_tmp < nx;
             transitionSamples_tmp++) {
          if ((segmentSamples[transitionSamples_tmp] < accum) || rtIsNaN(accum))
          {
            samplesAll[transitionSamples_tmp] =
              segmentSamples[transitionSamples_tmp];
          } else {
            samplesAll[transitionSamples_tmp] = accum;
          }
        }
      }

      r = this->PathSegments;
      r.parenReference();
      accumLengths.set_size((static_cast<int>(numSegments_tmp + 1.0)));
      accumLengths[0] = 0.0;
      for (s = 0; s < i; s++) {
        r = this->PathSegments;
        r.parenReference((static_cast<double>(s) + 1.0));
        r.get_Length(segLen_data, tmp_size);
        accumLengths[s + 1] = accumLengths[s] + segLen_data[0];
      }

      Path::discretizecg((samplesAll), (accumLengths), (segmentIndex));
      this->get_StartPose(b_this.data, b_this.size);
      poses.set_size(0, 3);
      b_directions.set_size(0, 1);
      for (int n = 0; n < i; n++) {
        transitionSamples_tmp = segmentIndex.size(0) - 1;
        nx = 0;
        for (s = 0; s <= transitionSamples_tmp; s++) {
          if (segmentIndex[s] == static_cast<double>(n) + 1.0) {
            nx++;
          }
        }

        r1.set_size(nx);
        nx = 0;
        for (s = 0; s <= transitionSamples_tmp; s++) {
          if (segmentIndex[s] == static_cast<double>(n) + 1.0) {
            r1[nx] = s + 1;
            nx++;
          }
        }

        nx = r1.size(0);
        segmentSamples.set_size(r1.size(0));
        for (i1 = 0; i1 < nx; i1++) {
          segmentSamples[i1] = samplesAll[r1[i1] - 1] - accumLengths[n];
        }

        r = this->PathSegments;
        r.parenReference((static_cast<double>(n) + 1.0));
        r.get_Length(segLen_data, tmp_size);
        i1 = segmentSamples.size(0);
        for (s = 0; s < i1; s++) {
          if ((segmentSamples[s] < segLen_data[0]) || rtIsNaN(segLen_data[0])) {
            accum = segmentSamples[s];
          } else {
            accum = segLen_data[0];
          }

          segmentSamples[s] = accum;
        }

        r = this->PathSegments;
        r.parenReference((static_cast<double>(n) + 1.0));
        r.interpolateInternal(segmentSamples, (n + 1U > 1U), segmentPoses,
                              segmentDirections);
        nx = poses.size(0);
        b_poses.set_size((poses.size(0) + segmentPoses.size(0)), 3);
        for (i1 = 0; i1 < 3; i1++) {
          for (transitionSamples_tmp = 0; transitionSamples_tmp < nx;
               transitionSamples_tmp++) {
            b_poses[transitionSamples_tmp + b_poses.size(0) * i1] =
              poses[transitionSamples_tmp + poses.size(0) * i1];
          }
        }

        nx = segmentPoses.size(0);
        for (i1 = 0; i1 < 3; i1++) {
          for (transitionSamples_tmp = 0; transitionSamples_tmp < nx;
               transitionSamples_tmp++) {
            b_poses[(transitionSamples_tmp + poses.size(0)) + b_poses.size(0) *
              i1] = segmentPoses[transitionSamples_tmp + segmentPoses.size(0) *
              i1];
          }
        }

        poses.set_size(b_poses.size(0), 3);
        nx = b_poses.size(0) * b_poses.size(1);
        for (i1 = 0; i1 < nx; i1++) {
          poses[i1] = b_poses[i1];
        }

        nx = b_directions.size(0);
        directions.set_size((b_directions.size(0) + segmentDirections.size(0)));
        for (i1 = 0; i1 < nx; i1++) {
          directions[i1] = b_directions[i1];
        }

        nx = segmentDirections.size(0);
        for (i1 = 0; i1 < nx; i1++) {
          directions[i1 + b_directions.size(0)] = segmentDirections[i1];
        }

        b_directions.set_size(directions.size(0), 1);
        nx = directions.size(0);
        for (i1 = 0; i1 < nx; i1++) {
          b_directions[i1] = directions[i1];
        }
      }

      directions.set_size(b_directions.size(0));
      nx = b_directions.size(0);
      for (i = 0; i < nx; i++) {
        directions[i] = b_directions[i];
      }
    }
  }
}

//
// File trailer for Path1.cpp
//
// [EOF]
//
