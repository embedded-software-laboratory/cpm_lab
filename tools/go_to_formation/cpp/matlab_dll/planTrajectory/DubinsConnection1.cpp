//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: DubinsConnection1.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jan-2021 13:31:05
//

// Include Files
#include "DubinsConnection1.h"
#include "DubinsBuiltins.h"
#include "NameValueParser.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions

//
// Arguments    : const double startPose[3]
//                const double goalPose[3]
//                double motionLengths[3]
//                char motionTypes[3]
// Return Type  : void
//
namespace mgen
{
  void e_driving_internal_planning_Dub::connectInternal(const double startPose[3],
    const double goalPose[3], double motionLengths[3], char motionTypes[3])
    const
  {
    d_matlabshared_autonomous_core_ parser;
    char unusedExpr[7];
    double cost;
    double tempMotionTypes[3];
    static const char cv[3] = { 'L', 'R', 'S' };

    parser.init();
    parser.parse();
    parser.parameterValue(unusedExpr);
    DubinsBuiltins::autonomousDubinsSegments((startPose), (goalPose),
      (this->MinTurningRadius), (this->DisabledPathTypesInternal), (&cost),
      (motionLengths), (tempMotionTypes));
    motionTypes[0] = cv[static_cast<int>(tempMotionTypes[0] + 1.0) - 1];
    motionTypes[1] = cv[static_cast<int>(tempMotionTypes[1] + 1.0) - 1];
    motionTypes[2] = cv[static_cast<int>(tempMotionTypes[2] + 1.0) - 1];
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  void e_driving_internal_planning_Dub::set_MinTurningRadius()
  {
    this->MinTurningRadius = 4.0;
  }

  //
  // Arguments    : double radius
  // Return Type  : void
  //
  void e_driving_internal_planning_Dub::set_MinTurningRadius(double radius)
  {
    this->MinTurningRadius = radius;
  }
}

//
// File trailer for DubinsConnection1.cpp
//
// [EOF]
//
