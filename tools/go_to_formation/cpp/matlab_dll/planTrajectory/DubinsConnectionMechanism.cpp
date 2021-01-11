//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: DubinsConnectionMechanism.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jan-2021 13:31:05
//

// Include Files
#include "DubinsConnectionMechanism.h"
#include "DubinsBuiltins.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions

//
// Arguments    : const double from[3]
//                const double to[3]
//                coder::array<double, 2U> &poses
// Return Type  : void
//
namespace mgen
{
  void c_matlabshared_planning_interna::b_interpolate(const double from[3],
    const double to[3], coder::array<double, 2U> &poses) const
  {
    DubinsBuiltins::b_autonomousDubinsInterpolate((from), (to),
      (this->ConnectionDistance), (this->NumSteps), (this->TurningRadius),
      (poses));
  }

  //
  // Arguments    : const double from[3]
  //                const double to[3]
  // Return Type  : double
  //
  double c_matlabshared_planning_interna::distance(const double from[3], const
    double to[3]) const
  {
    return DubinsBuiltins::autonomousDubinsDistance((from), (to),
      (this->TurningRadius));
  }

  //
  // Arguments    : const double from[3]
  //                const coder::array<double, 2U> &to
  //                coder::array<double, 1U> &d
  // Return Type  : void
  //
  void c_matlabshared_planning_interna::distance(const double from[3], const
    coder::array<double, 2U> &to, coder::array<double, 1U> &d) const
  {
    DubinsBuiltins::autonomousDubinsDistance((from), (to), (this->TurningRadius),
      (d));
  }

  //
  // Arguments    : const coder::array<double, 2U> &from
  //                const double to[3]
  //                coder::array<double, 1U> &d
  // Return Type  : void
  //
  void c_matlabshared_planning_interna::distance(const coder::array<double, 2U>
    &from, const double to[3], coder::array<double, 1U> &d) const
  {
    DubinsBuiltins::autonomousDubinsDistance((from), (to), (this->TurningRadius),
      (d));
  }

  //
  // Arguments    : const coder::array<double, 2U> &from
  //                const double to_data[]
  //                const int to_size[2]
  //                coder::array<double, 1U> &d
  // Return Type  : void
  //
  void c_matlabshared_planning_interna::distance(const coder::array<double, 2U>
    &from, const double to_data[], const int to_size[2], coder::array<double, 1U>
    &d) const
  {
    DubinsBuiltins::autonomousDubinsDistance((from), (to_data), (to_size),
      (this->TurningRadius), (d));
  }

  //
  // Arguments    : void
  // Return Type  : c_matlabshared_planning_interna *
  //
  c_matlabshared_planning_interna *c_matlabshared_planning_interna::init()
  {
    return this;
  }

  //
  // Arguments    : const double from[3]
  //                const double to_data[]
  //                coder::array<double, 2U> &poses
  // Return Type  : void
  //
  void c_matlabshared_planning_interna::interpolate(const double from[3], const
    double to_data[], coder::array<double, 2U> &poses) const
  {
    DubinsBuiltins::autonomousDubinsInterpolate((from), (to_data),
      (this->ConnectionDistance), (this->NumSteps), (this->TurningRadius),
      (poses));
  }
}

//
// File trailer for DubinsConnectionMechanism.cpp
//
// [EOF]
//
