//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: UniformPoseSampler.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 15-Feb-2021 16:41:56
//

// Include Files
#include "UniformPoseSampler.h"
#include "VehicleCostmapCodegen.h"
#include "VehicleCostmapImpl.h"
#include "planTrajectory.h"
#include "rand.h"
#include "rt_nonfinite.h"
#include <cstring>
#include <string.h>

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
namespace mgen
{
  void d_matlabshared_planning_interna::attemptResampling()
  {
    int numAttempts;
    boolean_T noFreePoses;
    boolean_T x[5000];
    numAttempts = 1;
    noFreePoses = true;
    while (noFreePoses && (numAttempts < 4)) {
      int k;
      boolean_T exitg1;
      this->fillPoseBuffer();
      std::memcpy(&x[0], &this->CollisionFree[0], 5000U * sizeof(boolean_T));
      noFreePoses = false;
      k = 0;
      exitg1 = false;
      while ((!exitg1) && (k < 5000)) {
        if (x[k]) {
          noFreePoses = true;
          exitg1 = true;
        } else {
          k++;
        }
      }

      noFreePoses = !noFreePoses;
      numAttempts++;
    }
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  void d_matlabshared_planning_interna::fillPoseBuffer()
  {
    static double r[15000];
    double a_idx_0;
    double a_idx_1;
    double a_idx_2;
    int k;
    static double c[15000];
    int c_tmp;
    b_rand(r);
    a_idx_0 = this->UpperLimits[0] - this->LowerLimits[0];
    a_idx_1 = this->UpperLimits[1] - this->LowerLimits[1];
    a_idx_2 = this->UpperLimits[2] - this->LowerLimits[2];
    for (k = 0; k < 5000; k++) {
      c[3 * k] = a_idx_0 * r[3 * k];
      c_tmp = 3 * k + 1;
      c[c_tmp] = a_idx_1 * r[c_tmp];
      c_tmp = 3 * k + 2;
      c[c_tmp] = a_idx_2 * r[c_tmp];
    }

    a_idx_0 = this->LowerLimits[0];
    a_idx_1 = this->LowerLimits[1];
    a_idx_2 = this->LowerLimits[2];
    for (k = 0; k < 5000; k++) {
      r[3 * k] = a_idx_0 + c[3 * k];
      c_tmp = 3 * k + 1;
      r[c_tmp] = a_idx_1 + c[c_tmp];
      c_tmp = 3 * k + 2;
      r[c_tmp] = a_idx_2 + c[c_tmp];
    }

    std::memcpy(&this->PoseBuffer[0], &r[0], 15000U * sizeof(double));
    for (c_tmp = 0; c_tmp < 3; c_tmp++) {
      for (k = 0; k < 5000; k++) {
        r[k + 5000 * c_tmp] = this->PoseBuffer[c_tmp + 3 * k];
      }
    }

    this->Costmap->checkFreePoses(r, this->CollisionFree);
    this->PoseIndex = 1.0;
  }

  //
  // Arguments    : double pose_data[]
  //                int pose_size[2]
  //                boolean_T *collisionFree
  // Return Type  : void
  //
  void d_matlabshared_planning_interna::sample(double pose_data[], int
    pose_size[2], boolean_T *collisionFree)
  {
    int collisionFree_tmp;
    boolean_T x[5000];
    collisionFree_tmp = static_cast<int>(this->PoseIndex);
    *collisionFree = this->CollisionFree[collisionFree_tmp - 1];
    if (*collisionFree) {
      pose_size[0] = 1;
      pose_size[1] = 3;
      collisionFree_tmp = 3 * (collisionFree_tmp - 1);
      pose_data[0] = this->PoseBuffer[collisionFree_tmp];
      pose_data[1] = this->PoseBuffer[collisionFree_tmp + 1];
      pose_data[2] = this->PoseBuffer[collisionFree_tmp + 2];
    } else {
      pose_size[0] = 3;
      pose_size[1] = 1;
      pose_data[0] = rtNaN;
      pose_data[1] = rtNaN;
      pose_data[2] = rtNaN;
    }

    this->PoseIndex++;
    if (this->PoseIndex > 5000.0) {
      boolean_T y;
      boolean_T exitg1;
      this->fillPoseBuffer();
      std::memcpy(&x[0], &this->CollisionFree[0], 5000U * sizeof(boolean_T));
      y = false;
      collisionFree_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (collisionFree_tmp < 5000)) {
        if (x[collisionFree_tmp]) {
          y = true;
          exitg1 = true;
        } else {
          collisionFree_tmp++;
        }
      }

      if (!y) {
        this->attemptResampling();
      }
    }
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  void d_matlabshared_planning_interna::configureCollisionChecker()
  {
    int k;
    static double b_this[15000];
    boolean_T x[5000];
    boolean_T y;
    boolean_T exitg1;
    for (k = 0; k < 3; k++) {
      for (int i = 0; i < 5000; i++) {
        b_this[i + 5000 * k] = this->PoseBuffer[k + 3 * i];
      }
    }

    this->Costmap->checkFreePoses(b_this, this->CollisionFree);
    std::memcpy(&x[0], &this->CollisionFree[0], 5000U * sizeof(boolean_T));
    y = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 5000)) {
      if (x[k]) {
        y = true;
        exitg1 = true;
      } else {
        k++;
      }
    }

    if (!y) {
      this->attemptResampling();
    }
  }

  //
  // Arguments    : c_driving_internal_costmap_Vehi *costmap
  // Return Type  : d_matlabshared_planning_interna *
  //
  d_matlabshared_planning_interna *d_matlabshared_planning_interna::init
    (c_driving_internal_costmap_Vehi *costmap)
  {
    d_matlabshared_planning_interna *this_;
    double worldExtent[4];
    double r[5000];
    this_ = this;
    this_->Costmap = costmap;
    costmap->get_MapExtent(worldExtent);
    this_->LowerLimits[0] = worldExtent[0];
    this_->LowerLimits[1] = worldExtent[2];
    this_->LowerLimits[2] = 0.0;
    this_->UpperLimits[0] = worldExtent[1];
    this_->UpperLimits[1] = worldExtent[3];
    this_->UpperLimits[2] = 6.2831853071795862;
    this_->fillPoseBuffer();
    c_rand(r);
    for (int i = 0; i < 5000; i++) {
      this_->GoalBiasBuffer[i] = r[i];
    }

    this_->GoalBiasIndex = 1.0;
    return this_;
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  void d_matlabshared_planning_interna::reset()
  {
    this->fillPoseBuffer();
    c_rand(this->GoalBiasBuffer);
    this->GoalBiasIndex = 1.0;
  }

  //
  // Arguments    : double pose_data[]
  //                int pose_size[2]
  // Return Type  : void
  //
  void d_matlabshared_planning_interna::sampleCollisionFree(double pose_data[],
    int pose_size[2])
  {
    boolean_T collisionFree;
    pose_size[0] = 3;
    pose_size[1] = 1;
    pose_data[0] = rtNaN;
    pose_data[1] = rtNaN;
    pose_data[2] = rtNaN;
    collisionFree = false;
    while (!collisionFree) {
      this->sample(pose_data, pose_size, (&collisionFree));
    }
  }

  //
  // Arguments    : void
  // Return Type  : double
  //
  double d_matlabshared_planning_interna::sampleGoalBias()
  {
    double goalBias;
    goalBias = this->GoalBiasBuffer[static_cast<int>(this->GoalBiasIndex) - 1];
    this->GoalBiasIndex++;
    if (this->GoalBiasIndex > 5000.0) {
      c_rand(this->GoalBiasBuffer);
      this->GoalBiasIndex = 1.0;
    }

    return goalBias;
  }
}

//
// File trailer for UniformPoseSampler.cpp
//
// [EOF]
//
