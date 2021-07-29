//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SqrtApproxNeighborSearcher.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 31-Mar-2021 23:01:38
//

// Include Files
#include "SqrtApproxNeighborSearcher.h"
#include "DubinsBuiltins.h"
#include "DubinsPathSegment.h"
#include "NeighborSearcher.h"
#include "extremeKElements.h"
#include "planTrajectory.h"
#include "planTrajectory_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <string.h>

// Function Definitions

//
// Arguments    : const double *numNodes
//                double *offset
//                double *step
// Return Type  : void
//
namespace mgen
{
  void e_matlabshared_planning_interna::computeOffsets(const double *numNodes,
    double *offset, double *step)
  {
    double x;
    double r;
    *step = std::floor(std::sqrt(*numNodes));
    x = this->Offset;
    r = x;
    if (*step == 0.0) {
      if (x == 0.0) {
        r = *step;
      }
    } else if (rtIsNaN(x) || rtIsNaN(*step) || rtIsInf(x)) {
      r = rtNaN;
    } else if (x == 0.0) {
      r = 0.0 / *step;
    } else if (rtIsInf(*step)) {
      if ((*step < 0.0) != (x < 0.0)) {
        r = *step;
      }
    } else {
      r = std::fmod(x, *step);
      if (r == 0.0) {
        r = *step * 0.0;
      } else {
        if ((x < 0.0) != (*step < 0.0)) {
          r += *step;
        }
      }
    }

    *offset = r + 1.0;
    this->Offset++;
  }

  //
  // Arguments    : const double nodeBuffer[30003]
  //                double numNodes
  //                const double node_data[]
  //                const int node_size[2]
  //                double nearestNode[3]
  //                double *nearestId
  // Return Type  : void
  //
  void e_matlabshared_planning_interna::b_nearest(const double nodeBuffer[30003],
    double numNodes, const double node_data[], const int node_size[2], double
    nearestNode[3], double *nearestId)
  {
    double offset;
    double step;
    int i;
    int n;
    int k;
    int loop_ub;
    coder::array<double, 2U> b_nodeBuffer;
    coder::array<double, 1U> varargin_1;
    this->computeOffsets((&numNodes), (&offset), (&step));
    if ((step == 0.0) || (((step > 0.0) && (offset > numNodes)) || ((0.0 > step)
          && (numNodes > offset)))) {
      i = 0;
      n = 1;
      k = -1;
    } else {
      i = static_cast<int>(offset) - 1;
      n = static_cast<int>(step);
      k = static_cast<int>(numNodes) - 1;
    }

    loop_ub = div_s32_floor(k - i, n);
    b_nodeBuffer.set_size((loop_ub + 1), 3);
    for (k = 0; k < 3; k++) {
      for (int i1 = 0; i1 <= loop_ub; i1++) {
        b_nodeBuffer[i1 + b_nodeBuffer.size(0) * k] = nodeBuffer[(i + n * i1) +
          10001 * k];
      }
    }

    this->distance(b_nodeBuffer, node_data, node_size, varargin_1);
    n = varargin_1.size(0);
    if (varargin_1.size(0) <= 2) {
      if (varargin_1.size(0) == 1) {
        loop_ub = 1;
      } else if ((varargin_1[0] > varargin_1[1]) || (rtIsNaN(varargin_1[0]) && (
                   !rtIsNaN(varargin_1[1])))) {
        loop_ub = 2;
      } else {
        loop_ub = 1;
      }
    } else {
      if (!rtIsNaN(varargin_1[0])) {
        loop_ub = 1;
      } else {
        boolean_T exitg1;
        loop_ub = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= varargin_1.size(0))) {
          if (!rtIsNaN(varargin_1[k - 1])) {
            loop_ub = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (loop_ub == 0) {
        loop_ub = 1;
      } else {
        double ex;
        ex = varargin_1[loop_ub - 1];
        i = loop_ub + 1;
        for (k = i; k <= n; k++) {
          if (ex > varargin_1[k - 1]) {
            ex = varargin_1[k - 1];
            loop_ub = k;
          }
        }
      }
    }

    *nearestId = offset + (static_cast<double>(loop_ub) - 1.0) * step;
    n = static_cast<int>(*nearestId);
    nearestNode[0] = nodeBuffer[n - 1];
    nearestNode[1] = nodeBuffer[n + 10000];
    nearestNode[2] = nodeBuffer[n + 20001];
  }

  //
  // Arguments    : c_matlabshared_planning_interna *varargin_1
  // Return Type  : e_matlabshared_planning_interna *
  //
  e_matlabshared_planning_interna *e_matlabshared_planning_interna::init
    (c_matlabshared_planning_interna *varargin_1)
  {
    e_matlabshared_planning_interna *this_;
    this_ = this;
    this_->ConnectionMechanism = varargin_1;
    this_->reset();
    return this_;
  }

  //
  // Arguments    : const double nodeBuffer[30003]
  //                double numNodes
  //                const double node[3]
  //                double K
  //                coder::array<double, 2U> &nearNodes
  //                coder::array<double, 1U> &nearIds
  // Return Type  : void
  //
  void e_matlabshared_planning_interna::near(const double nodeBuffer[30003],
    double numNodes, const double node[3], double K, coder::array<double, 2U>
    &nearNodes, coder::array<double, 1U> &nearIds)
  {
    double offset;
    double step;
    int i;
    int i1;
    int i2;
    int loop_ub;
    coder::array<double, 2U> b_nodeBuffer;
    coder::array<double, 1U> a;
    int idx_data[10002];
    int idx_size[1];
    coder::array<double, 1U> b;
    this->computeOffsets((&numNodes), (&offset), (&step));
    if ((step == 0.0) || (((step > 0.0) && (offset > numNodes)) || ((0.0 > step)
          && (numNodes > offset)))) {
      i = 0;
      i1 = 1;
      i2 = -1;
    } else {
      i = static_cast<int>(offset) - 1;
      i1 = static_cast<int>(step);
      i2 = static_cast<int>(numNodes) - 1;
    }

    loop_ub = div_s32_floor(i2 - i, i1);
    b_nodeBuffer.set_size((loop_ub + 1), 3);
    for (i2 = 0; i2 < 3; i2++) {
      for (int i3 = 0; i3 <= loop_ub; i3++) {
        b_nodeBuffer[i3 + b_nodeBuffer.size(0) * i2] = nodeBuffer[(i + i1 * i3)
          + 10001 * i2];
      }
    }

    this->distance(b_nodeBuffer, node, a);
    if (K <= a.size(0)) {
      loop_ub = static_cast<int>(K);
    } else {
      loop_ub = a.size(0);
    }

    exkib(a, loop_ub, idx_data, idx_size, b);
    nearIds.set_size(idx_size[0]);
    loop_ub = idx_size[0];
    for (i = 0; i < loop_ub; i++) {
      nearIds[i] = offset + (static_cast<double>(idx_data[i]) - 1.0) * step;
    }

    nearNodes.set_size(nearIds.size(0), 3);
    loop_ub = nearIds.size(0);
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < loop_ub; i1++) {
        nearNodes[i1 + nearNodes.size(0) * i] = nodeBuffer[(static_cast<int>
          (nearIds[i1]) + 10001 * i) - 1];
      }
    }
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  void e_matlabshared_planning_interna::reset()
  {
    this->Offset = 0.0;
  }
}

//
// File trailer for SqrtApproxNeighborSearcher.cpp
//
// [EOF]
//
