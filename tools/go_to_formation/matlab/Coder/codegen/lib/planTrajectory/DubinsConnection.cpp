//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: DubinsConnection.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Mar-2021 12:18:40
//

// Include Files
#include "DubinsConnection.h"
#include "DubinsConnection1.h"
#include "NameValueParser.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions

//
// Arguments    : mgen::e_driving_internal_planning_Dub *iobj_0
// Return Type  : mgen::e_driving_internal_planning_Dub *
//
namespace mgen
{
  e_driving_internal_planning_Dub *e_driving_internal_planning_Dub::create
    (e_driving_internal_planning_Dub *iobj_0)
  {
    e_driving_internal_planning_Dub *this_;
    c_matlabshared_autonomous_core_ parser;
    double minTurningRadius;
    this_ = iobj_0;
    parser.init();
    parser.parse();
    minTurningRadius = parser.parameterValue();
    iobj_0->set_MinTurningRadius(minTurningRadius);
    iobj_0->DisabledPathTypesInternal.set_size(0, 0);
    iobj_0->set_MinTurningRadius();
    return this_;
  }
}

//
// File trailer for DubinsConnection.cpp
//
// [EOF]
//
