//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RRTPlanner.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 31-Mar-2021 23:01:38
//

// Include Files
#include "RRTPlanner.h"
#include "DubinsBuiltins.h"
#include "DubinsConnectionMechanism.h"
#include "DubinsPathSegment.h"
#include "Path1.h"
#include "RRTTree.h"
#include "SqrtApproxNeighborSearcher.h"
#include "UniformPoseSampler.h"
#include "VehicleCostmapImpl.h"
#include "angleUtilities.h"
#include "pathToTrajectory.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <string.h>

// Function Definitions

//
// Arguments    : const coder::array<double, 2U> &nearPoses
//                const coder::array<double, 1U> &nearIds
//                const double nearestPose[3]
//                double nearestId
//                const double newPose[3]
//                double newId
// Return Type  : void
//
namespace mgen
{
  void g_matlabshared_planning_interna::findMinCostPath(const coder::array<
    double, 2U> &nearPoses, const coder::array<double, 1U> &nearIds, const
    double nearestPose[3], double nearestId, const double newPose[3], double
    newId)
  {
    double minCost;
    double minCostId;
    double maxDist;
    coder::array<double, 1U> distances;
    int i;
    double b_nearPoses[3];
    coder::array<double, 2U> posesInterp;
    coder::array<boolean_T, 1U> b_free;
    minCost = this->Tree.costTo(nearestId) + this->ConnectionMechanism->distance
      (nearestPose, newPose);
    minCostId = nearestId;
    maxDist = this->ConnectionMechanism->ConnectionDistance;
    this->ConnectionMechanism->distance(nearPoses, newPose, distances);
    i = nearIds.size(0);
    for (int n = 0; n < i; n++) {
      double nearCost;
      nearCost = this->Tree.costTo(nearIds[n]) + distances[n];
      if ((nearCost < minCost) && (distances[n] <= maxDist)) {
        boolean_T y;
        int ix;
        boolean_T exitg1;
        b_nearPoses[0] = nearPoses[n];
        b_nearPoses[1] = nearPoses[n + nearPoses.size(0)];
        b_nearPoses[2] = nearPoses[n + nearPoses.size(0) * 2];
        this->ConnectionMechanism->b_interpolate(b_nearPoses, newPose,
          posesInterp);
        this->Costmap->c_checkFreePoses(posesInterp, b_free);
        y = true;
        ix = 1;
        exitg1 = false;
        while ((!exitg1) && (ix <= b_free.size(0))) {
          if (!b_free[ix - 1]) {
            y = false;
            exitg1 = true;
          } else {
            ix++;
          }
        }

        if (y) {
          minCost = nearCost;
          minCostId = nearIds[n];
        }
      }
    }

    this->Tree.addEdge(minCostId, newId);
  }

  //
  // Arguments    : const double pose[3]
  // Return Type  : boolean_T
  //
  boolean_T g_matlabshared_planning_interna::inGoalRegion(const double pose[3])
    const
  {
    boolean_T TF;
    if ((std::abs(pose[0] - this->GoalPose[0]) <= this->GoalTolerance[0]) &&
        (std::abs(pose[1] - this->GoalPose[1]) <= this->GoalTolerance[1]) &&
        (std::abs(angleUtilities::angdiff((pose[2]), (this->GoalPose[2]))) <=
         this->GoalTolerance[2])) {
      TF = true;
    } else {
      TF = false;
    }

    return TF;
  }

  //
  // Arguments    : const coder::array<double, 2U> &nearPoses
  //                const coder::array<double, 1U> &nearIds
  //                const double newPose[3]
  //                double newId
  // Return Type  : void
  //
  void g_matlabshared_planning_interna::rewireTree(const coder::array<double, 2U>
    &nearPoses, const coder::array<double, 1U> &nearIds, const double newPose[3],
    double newId)
  {
    f_matlabshared_planning_interna *treeLocal;
    double thetaTol;
    double newCost;
    double maxDist;
    coder::array<double, 1U> forwardDistances;
    coder::array<double, 1U> reverseDistances;
    int i;
    double b_nearPoses[3];
    coder::array<double, 2U> posesInterp;
    coder::array<boolean_T, 1U> b_free;
    double b_y;
    treeLocal = &this->Tree;
    thetaTol = this->GoalTolerance[2];
    newCost = this->Tree.costTo(newId);
    maxDist = this->ConnectionMechanism->ConnectionDistance;
    this->ConnectionMechanism->distance(nearPoses, newPose, forwardDistances);
    this->ConnectionMechanism->distance(newPose, nearPoses, reverseDistances);
    i = nearIds.size(0);
    for (int n = 0; n < i; n++) {
      double nearCost;
      nearCost = treeLocal->costTo(nearIds[n]);
      if ((newCost + reverseDistances[n] < nearCost) && (forwardDistances[n] <=
           maxDist)) {
        boolean_T y;
        int ix;
        boolean_T exitg1;
        b_nearPoses[0] = nearPoses[n];
        b_nearPoses[1] = nearPoses[n + nearPoses.size(0)];
        b_nearPoses[2] = nearPoses[n + nearPoses.size(0) * 2];
        this->ConnectionMechanism->b_interpolate(newPose, b_nearPoses,
          posesInterp);
        this->Costmap->c_checkFreePoses(posesInterp, b_free);
        y = true;
        ix = 1;
        exitg1 = false;
        while ((!exitg1) && (ix <= b_free.size(0))) {
          if (!b_free[ix - 1]) {
            y = false;
            exitg1 = true;
          } else {
            ix++;
          }
        }

        if (y) {
          double absxk;
          double t;
          nearCost = 3.3121686421112381E-170;
          absxk = std::abs(posesInterp[posesInterp.size(0) - 1] - nearPoses[n]);
          if (absxk > 3.3121686421112381E-170) {
            b_y = 1.0;
            nearCost = absxk;
          } else {
            t = absxk / 3.3121686421112381E-170;
            b_y = t * t;
          }

          absxk = std::abs(posesInterp[(posesInterp.size(0) + posesInterp.size(0))
                           - 1] - nearPoses[n + nearPoses.size(0)]);
          if (absxk > nearCost) {
            t = nearCost / absxk;
            b_y = b_y * t * t + 1.0;
            nearCost = absxk;
          } else {
            t = absxk / nearCost;
            b_y += t * t;
          }

          b_y = nearCost * std::sqrt(b_y);
          if ((b_y <= 0.01) && (std::abs(angleUtilities::angdiff((posesInterp
                  [(posesInterp.size(0) + posesInterp.size(0) * 2) - 1]),
                 (nearPoses[n + nearPoses.size(0) * 2]))) <= thetaTol)) {
            treeLocal->replaceParent(nearIds[n], newId);
          }
        }
      }
    }
  }

  //
  // Arguments    : c_driving_internal_costmap_Vehi *costmap
  //                const double goalTolerance[3]
  //                e_matlabshared_planning_interna *iobj_0
  //                c_matlabshared_planning_interna *iobj_1
  // Return Type  : g_matlabshared_planning_interna *
  //
  g_matlabshared_planning_interna *g_matlabshared_planning_interna::init
    (c_driving_internal_costmap_Vehi *costmap, const double goalTolerance[3],
     e_matlabshared_planning_interna *iobj_0, c_matlabshared_planning_interna
     *iobj_1)
  {
    g_matlabshared_planning_interna *this_;
    c_matlabshared_planning_interna *connMech;
    e_matlabshared_planning_interna *neighborSearcher;
    this_ = this;
    this_->Costmap = costmap;
    this_->Sampler.init(this_->Costmap);
    connMech = iobj_1->init();
    connMech->ConnectionDistance = 5.0;
    connMech->TurningRadius = 0.4;
    connMech->NumSteps = 2500.0;
    this_->ConnectionMechanism = connMech;
    neighborSearcher = iobj_0->init(connMech);
    this_->MinIterations = 100.0;
    this_->MaxIterations = 10000.0;
    this_->GoalTolerance[0] = goalTolerance[0];
    this_->GoalTolerance[1] = goalTolerance[1];
    this_->GoalTolerance[2] = goalTolerance[2];
    this_->GoalBias = 0.1;
    this_->Tree.init(neighborSearcher);
    return this_;
  }

  //
  // Arguments    : const double startPose[3]
  //                const double goalPose[3]
  //                coder::array<double, 2U> &varargout_1
  // Return Type  : void
  //
  void g_matlabshared_planning_interna::planPath(const double startPose[3],
    const double goalPose[3], coder::array<double, 2U> &varargout_1)
  {
    coder::array<double, 2U> goalNodes;
    int bestGoalNode_size[2];
    double bestCost;
    double maxIter;
    int n;
    boolean_T exitg1;
    double bestGoalNode_data[1];
    static unsigned int path_data[10001];
    int randPose_size[2];
    double cost_data[1];
    int cost_size[2];
    double randPose_data[9];
    coder::array<double, 2U> r;
    double nearestPose[3];
    double planCost;
    int ix;
    coder::array<double, 2U> posesInterp;
    int i;
    coder::array<boolean_T, 1U> b_free;
    double b_posesInterp[3];
    coder::array<double, 2U> nearPoses;
    coder::array<double, 1U> nearIds;
    this->StartPose[0] = startPose[0];
    this->StartPose[1] = startPose[1];
    this->StartPose[2] = startPose[2];
    this->GoalPose[0] = goalPose[0];
    this->GoalPose[1] = goalPose[1];
    this->GoalPose[2] = goalPose[2];
    this->Tree.reset();
    this->Sampler.reset();
    varargout_1.set_size(0, 3);
    goalNodes.set_size(1, 0);
    bestGoalNode_size[0] = 0;
    bestGoalNode_size[1] = 0;
    bestCost = rtInf;
    this->Sampler.configureCollisionChecker();
    this->Tree.addNode(startPose);
    maxIter = this->MaxIterations;
    n = 0;
    exitg1 = false;
    while ((!exitg1) && (n <= static_cast<int>(maxIter) - 1)) {
      boolean_T y;
      boolean_T exitg2;
      if (this->Sampler.sampleGoalBias() > this->GoalBias) {
        this->Sampler.sampleCollisionFree(randPose_data, randPose_size);
      } else {
        randPose_size[0] = 1;
        randPose_size[1] = 3;
        randPose_data[0] = this->GoalPose[0];
        randPose_data[1] = this->GoalPose[1];
        randPose_data[2] = this->GoalPose[2];
      }

      this->Tree.b_nearest(randPose_data, randPose_size, nearestPose, (&planCost));
      this->ConnectionMechanism->interpolate(nearestPose, randPose_data,
        posesInterp);
      this->Costmap->c_checkFreePoses(posesInterp, b_free);
      y = true;
      ix = 1;
      exitg2 = false;
      while ((!exitg2) && (ix <= b_free.size(0))) {
        if (!b_free[ix - 1]) {
          y = false;
          exitg2 = true;
        } else {
          ix++;
        }
      }

      if (!y) {
        n++;
      } else {
        double newId;
        b_posesInterp[0] = posesInterp[posesInterp.size(0) - 1];
        b_posesInterp[1] = posesInterp[(posesInterp.size(0) + posesInterp.size(0))
          - 1];
        b_posesInterp[2] = posesInterp[(posesInterp.size(0) + posesInterp.size(0)
          * 2) - 1];
        this->Tree.near(b_posesInterp, nearPoses, nearIds);
        b_posesInterp[0] = posesInterp[posesInterp.size(0) - 1];
        b_posesInterp[1] = posesInterp[(posesInterp.size(0) + posesInterp.size(0))
          - 1];
        b_posesInterp[2] = posesInterp[(posesInterp.size(0) + posesInterp.size(0)
          * 2) - 1];
        newId = this->Tree.b_addNode(b_posesInterp);
        b_posesInterp[0] = posesInterp[posesInterp.size(0) - 1];
        b_posesInterp[1] = posesInterp[(posesInterp.size(0) + posesInterp.size(0))
          - 1];
        b_posesInterp[2] = posesInterp[(posesInterp.size(0) + posesInterp.size(0)
          * 2) - 1];
        this->findMinCostPath(nearPoses, nearIds, nearestPose, planCost,
                              b_posesInterp, newId);
        b_posesInterp[0] = posesInterp[posesInterp.size(0) - 1];
        b_posesInterp[1] = posesInterp[(posesInterp.size(0) + posesInterp.size(0))
          - 1];
        b_posesInterp[2] = posesInterp[(posesInterp.size(0) + posesInterp.size(0)
          * 2) - 1];
        this->rewireTree(nearPoses, nearIds, b_posesInterp, newId);
        b_posesInterp[0] = posesInterp[posesInterp.size(0) - 1];
        b_posesInterp[1] = posesInterp[(posesInterp.size(0) + posesInterp.size(0))
          - 1];
        b_posesInterp[2] = posesInterp[(posesInterp.size(0) + posesInterp.size(0)
          * 2) - 1];
        if (this->inGoalRegion(b_posesInterp)) {
          i = goalNodes.size(1);
          goalNodes.set_size(goalNodes.size(0), (goalNodes.size(1) + 1));
          goalNodes[i] = newId;
          planCost = this->Tree.costTo(newId);
          if (planCost < bestCost) {
            bestGoalNode_size[0] = 1;
            bestGoalNode_size[1] = 1;
            bestGoalNode_data[0] = newId;
            bestCost = planCost;
          }

          y = (static_cast<double>(n) + 1.0 >= this->MinIterations);
          if (y) {
            exitg1 = true;
          } else {
            n++;
          }
        } else {
          n++;
        }
      }
    }

    if (goalNodes.size(1) != 0) {
      this->Tree.shortestPathFromRoot(bestGoalNode_data, bestGoalNode_size,
        goalNodes, path_data, randPose_size, cost_data, cost_size);
      this->Tree.get_Nodes(r);
      varargout_1.set_size(randPose_size[1], 3);
      ix = randPose_size[1];
      for (i = 0; i < 3; i++) {
        for (n = 0; n < ix; n++) {
          varargout_1[n + varargout_1.size(0) * i] = r[(static_cast<int>
            (path_data[n]) + r.size(0) * i) - 1];
        }
      }
    }
  }
}

//
// File trailer for RRTPlanner.cpp
//
// [EOF]
//
