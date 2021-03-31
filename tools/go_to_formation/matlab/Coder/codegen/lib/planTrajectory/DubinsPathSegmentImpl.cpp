//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: DubinsPathSegmentImpl.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 31-Mar-2021 23:01:38
//

// Include Files
#include "DubinsPathSegmentImpl.h"
#include "DubinsConnection1.h"
#include "angleUtilities.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions

//
// Arguments    : const mgen::e_driving_internal_planning_Dub *varargin_1
//                const double varargin_2[3]
//                const double varargin_3[3]
// Return Type  : void
//
namespace mgen
{
  void c_driving_internal_planning_Dub::init(const
    e_driving_internal_planning_Dub *varargin_1, const double varargin_2[3],
    const double varargin_3[3])
  {
    double startPose[3];
    double goalPose[3];
    double motionLengths[3];
    char motionTypes[3];
    startPose[0] = varargin_2[0];
    startPose[1] = varargin_2[1];
    startPose[2] = varargin_2[2];
    angleUtilities::convertAndWrapTo2Pi((&startPose[2]));
    goalPose[0] = varargin_3[0];
    goalPose[1] = varargin_3[1];
    goalPose[2] = varargin_3[2];
    angleUtilities::convertAndWrapTo2Pi((&goalPose[2]));
    this->MinTurningRadius = varargin_1->MinTurningRadius;
    this->StartPoseInternal[0] = startPose[0];
    this->StartPoseInternal[1] = startPose[1];
    this->StartPoseInternal[2] = startPose[2];
    angleUtilities::wrapTo2Pi((&this->StartPoseInternal[2]));
    this->GoalPoseInternal[0] = goalPose[0];
    this->GoalPoseInternal[1] = goalPose[1];
    this->GoalPoseInternal[2] = goalPose[2];
    angleUtilities::wrapTo2Pi((&this->GoalPoseInternal[2]));
    varargin_1->connectInternal(startPose, goalPose, motionLengths, motionTypes);
    this->MotionLengths[0] = motionLengths[0];
    this->MotionTypes[0] = motionTypes[0];
    this->MotionLengths[1] = motionLengths[1];
    this->MotionTypes[1] = motionTypes[1];
    this->MotionLengths[2] = motionLengths[2];
    this->MotionTypes[2] = motionTypes[2];
  }

  //
  // Arguments    : const e_driving_internal_planning_Dub *varargin_1
  // Return Type  : void
  //
  void c_driving_internal_planning_Dub::init(const
    e_driving_internal_planning_Dub *varargin_1)
  {
    double d;
    this->StartPoseInternal[0] = 0.0;
    this->StartPoseInternal[1] = 0.0;
    d = 0.0;
    angleUtilities::convertAndWrapTo2Pi((&d));
    this->MinTurningRadius = varargin_1->MinTurningRadius;
    this->StartPoseInternal[2] = d;
    angleUtilities::wrapTo2Pi((&this->StartPoseInternal[2]));
    this->GoalPoseInternal[0] = 0.0;
    this->GoalPoseInternal[1] = 0.0;
    this->GoalPoseInternal[2] = d;
    angleUtilities::wrapTo2Pi((&this->GoalPoseInternal[2]));
    this->MotionLengths[0] = 0.0;
    this->MotionTypes[0] = 'L';
    this->MotionLengths[1] = 0.0;
    this->MotionTypes[1] = 'S';
    this->MotionLengths[2] = 0.0;
    this->MotionTypes[2] = 'L';
  }

  //
  // Arguments    : const e_driving_internal_planning_Dub *varargin_1
  //                const double varargin_2[3]
  //                const double varargin_3[3]
  //                c_driving_internal_planning_Dub *b_this
  // Return Type  : void
  //
  void c_driving_internal_planning_Dub::create(const
    e_driving_internal_planning_Dub *varargin_1, const double varargin_2[3],
    const double varargin_3[3], c_driving_internal_planning_Dub *b_this)
  {
    b_this->init(varargin_1, varargin_2, varargin_3);
  }

  //
  // Arguments    : const e_driving_internal_planning_Dub *varargin_1
  //                c_driving_internal_planning_Dub *b_this
  // Return Type  : void
  //
  void c_driving_internal_planning_Dub::create(const
    e_driving_internal_planning_Dub *varargin_1, c_driving_internal_planning_Dub
    *b_this)
  {
    b_this->init(varargin_1);
  }

  //
  // Arguments    : double startPose[3]
  // Return Type  : void
  //
  void c_driving_internal_planning_Dub::get_StartPose(double startPose[3]) const
  {
    startPose[0] = this->StartPoseInternal[0];
    startPose[1] = this->StartPoseInternal[1];
    startPose[2] = 57.295779513082323 * this->StartPoseInternal[2];
  }
}

//
// File trailer for DubinsPathSegmentImpl.cpp
//
// [EOF]
//
