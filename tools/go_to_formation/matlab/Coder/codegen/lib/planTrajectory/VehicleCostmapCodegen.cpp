//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: VehicleCostmapCodegen.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 15-Feb-2021 16:41:56
//

// Include Files
#include "VehicleCostmapCodegen.h"
#include "imdilate.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions

//
// Arguments    : const double varargin_1[180000]
//                const double varargin_5[2]
//                double varargin_9
//                double varargin_11
//                const boolean_T varargin_13[180000]
//                const boolean_T varargin_15[180000]
// Return Type  : mgen::c_driving_internal_costmap_Vehi *
//
namespace mgen
{
  c_driving_internal_costmap_Vehi *c_driving_internal_costmap_Vehi::b_init(const
    double varargin_1[180000], const double varargin_5[2], double varargin_9,
    double varargin_11, const boolean_T varargin_13[180000], const boolean_T
    varargin_15[180000])
  {
    c_driving_internal_costmap_Vehi *this_;
    int i;
    this_ = this;
    this_->MapLocation[0] = varargin_5[0];
    this_->MapLocation[1] = varargin_5[1];
    for (i = 0; i < 180000; i++) {
      this_->Costmap[i] = varargin_1[i];
    }

    this_->pFreeThreshold = varargin_9;
    this_->pOccupiedThreshold = varargin_11;
    this_->CollisionCheckOffsets[0] = 0.025;
    this_->CollisionCheckOffsets[1] = 0.08;
    this_->CollisionCheckOffsets[2] = 0.135;
    for (i = 0; i < 180000; i++) {
      this_->FreeMap[i] = varargin_13[i];
    }

    for (i = 0; i < 180000; i++) {
      this_->OccupiedMap[i] = varargin_15[i];
    }

    return this_;
  }

  //
  // Arguments    : void
  // Return Type  : double
  //
  double c_driving_internal_costmap_Vehi::get_FreeThreshold() const
  {
    return this->pFreeThreshold;
  }

  //
  // Arguments    : void
  // Return Type  : double
  //
  double c_driving_internal_costmap_Vehi::get_OccupiedThreshold() const
  {
    return this->pOccupiedThreshold;
  }

  //
  // Arguments    : c_driving_internal_costmap_Vehi *iobj_0
  // Return Type  : c_driving_internal_costmap_Vehi *
  //
  c_driving_internal_costmap_Vehi *c_driving_internal_costmap_Vehi::copy
    (c_driving_internal_costmap_Vehi *iobj_0) const
  {
    return iobj_0->b_init(this->Costmap, this->MapLocation,
                          this->get_FreeThreshold(), this->get_OccupiedThreshold
                          (), this->FreeMap, this->OccupiedMap);
  }

  //
  // Arguments    : double mapExtent[4]
  // Return Type  : void
  //
  void c_driving_internal_costmap_Vehi::get_MapExtent(double mapExtent[4]) const
  {
    double mapLocation_idx_0;
    double mapLocation_idx_1;
    mapLocation_idx_0 = this->MapLocation[0];
    mapLocation_idx_1 = this->MapLocation[1];
    mapExtent[0] = mapLocation_idx_0;
    mapExtent[1] = mapLocation_idx_0 + 4.5;
    mapExtent[2] = mapLocation_idx_1;
    mapExtent[3] = mapLocation_idx_1 + 4.0;
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  void c_driving_internal_costmap_Vehi::inflate()
  {
    int i;
    boolean_T b_this[180000];
    for (i = 0; i < 180000; i++) {
      b_this[i] = (this->Costmap[i] > this->pOccupiedThreshold);
    }

    imdilate(b_this, this->OccupiedMap);
    for (i = 0; i < 180000; i++) {
      this->FreeMap[i] = ((this->Costmap[i] < this->pFreeThreshold) &&
                          (!this->OccupiedMap[i]));
    }
  }

  //
  // Arguments    : void
  // Return Type  : c_driving_internal_costmap_Vehi *
  //
  c_driving_internal_costmap_Vehi *c_driving_internal_costmap_Vehi::init()
  {
    c_driving_internal_costmap_Vehi *this_;
    this_ = this;
    this_->MapLocation[0] = 0.0;
    this_->MapLocation[1] = 0.0;
    for (int i = 0; i < 180000; i++) {
      this_->Costmap[i] = 0.0;
    }

    this_->pFreeThreshold = 0.2;
    this_->pOccupiedThreshold = 0.65;
    this_->CollisionCheckOffsets[0] = 0.025;
    this_->CollisionCheckOffsets[1] = 0.08;
    this_->CollisionCheckOffsets[2] = 0.135;
    this_->inflate();
    return this_;
  }
}

//
// File trailer for VehicleCostmapCodegen.cpp
//
// [EOF]
//
