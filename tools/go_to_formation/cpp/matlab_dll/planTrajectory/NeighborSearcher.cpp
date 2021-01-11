//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: NeighborSearcher.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jan-2021 13:31:05
//

// Include Files
#include "NeighborSearcher.h"
#include "DubinsConnectionMechanism.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions

//
// Arguments    : const double from[3]
//                const double to[3]
// Return Type  : double
//
namespace mgen
{
  double e_matlabshared_planning_interna::distance(const double from[3], const
    double to[3]) const
  {
    return this->ConnectionMechanism->distance(from, to);
  }

  //
  // Arguments    : const coder::array<double, 2U> &from
  //                const double to[3]
  //                coder::array<double, 1U> &d
  // Return Type  : void
  //
  void e_matlabshared_planning_interna::distance(const coder::array<double, 2U>
    &from, const double to[3], coder::array<double, 1U> &d) const
  {
    this->ConnectionMechanism->distance(from, to, d);
  }

  //
  // Arguments    : const coder::array<double, 2U> &from
  //                const double to_data[]
  //                const int to_size[2]
  //                coder::array<double, 1U> &d
  // Return Type  : void
  //
  void e_matlabshared_planning_interna::distance(const coder::array<double, 2U>
    &from, const double to_data[], const int to_size[2], coder::array<double, 1U>
    &d) const
  {
    this->ConnectionMechanism->distance(from, to_data, to_size, d);
  }
}

//
// File trailer for NeighborSearcher.cpp
//
// [EOF]
//
