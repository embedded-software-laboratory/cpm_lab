//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: OneDimArrayBehavior.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 15-Feb-2021 16:41:56
//

// Include Files
#include "OneDimArrayBehavior.h"
#include "DubinsPathSegmentCodegen.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions

//
// Arguments    : void
// Return Type  : boolean_T
//
namespace mgen
{
  boolean_T d_driving_internal_planning_Dub::isrow() const
  {
    return this->Data.size(0) == 1;
  }

  //
  // Arguments    : void
  // Return Type  : boolean_T
  //
  boolean_T d_driving_internal_planning_Dub::isempty() const
  {
    return this->numel() == 0.0;
  }

  //
  // Arguments    : void
  // Return Type  : double
  //
  double d_driving_internal_planning_Dub::length() const
  {
    return this->numel();
  }

  //
  // Arguments    : void
  // Return Type  : double
  //
  double d_driving_internal_planning_Dub::numel() const
  {
    return this->Data.size(0) * this->Data.size(1);
  }

  //
  // Arguments    : const d_driving_internal_planning_Dub *rhs
  //                double idx
  // Return Type  : void
  //
  void d_driving_internal_planning_Dub::parenAssign(const
    d_driving_internal_planning_Dub *rhs, double idx)
  {
    double d;
    coder::array<c_driving_internal_planning_Dub, 2U> data;
    d = this->numel();
    if (idx > d) {
      int i;
      int n;
      if (this->isrow()) {
        data.set_size(1, (static_cast<int>(idx)));
      } else {
        data.set_size((static_cast<int>(idx)), 1);
      }

      i = static_cast<int>(d);
      for (n = 0; n < i; n++) {
        data[n] = this->Data[n];
      }

      data[static_cast<int>(idx) - 1] = rhs->Data[0];
      this->Data.set_size(data.size(0), data.size(1));
      n = data.size(0) * data.size(1);
      for (i = 0; i < n; i++) {
        this->Data[i] = data[i];
      }
    } else {
      this->Data[static_cast<int>(idx) - 1] = rhs->Data[0];
    }
  }

  //
  // Arguments    : double idx
  // Return Type  : void
  //
  void d_driving_internal_planning_Dub::parenReference(double idx)
  {
    c_driving_internal_planning_Dub data_data_idx_0;
    data_data_idx_0 = this->Data[static_cast<int>(idx) - 1];
    this->Data.set_size(1, 1);
    this->Data[0] = data_data_idx_0;
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  void d_driving_internal_planning_Dub::parenReference()
  {
    c_driving_internal_planning_Dub data_data_idx_0;
    data_data_idx_0 = this->Data[0];
    this->Data.set_size(1, 1);
    this->Data[0] = data_data_idx_0;
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  void d_driving_internal_planning_Dub::repmat()
  {
    this->Data.set_size(0, 0);
  }
}

//
// File trailer for OneDimArrayBehavior.cpp
//
// [EOF]
//
